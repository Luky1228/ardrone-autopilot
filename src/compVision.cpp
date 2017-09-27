
/*
    compVision.cpp

    -------------------------------------------------------------------
    | Provides the computer vision algorithm that processes           |
    |  the image received from the imageHandler.cpp.                  |
    |                                                                 |
    | Sends back extracted information about the circles and the box. |
    |                                                                 |
    | Target detection algorithm is based on this OpenCV tutorial     |
    |  from the documentation: https://goo.gl/uFZrqR                  |
    |                                                                 |
    -------------------------------------------------------------------

*/
#define _USE_MATH_DEFINES
#include <algorithm>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs.hpp>
#include "imgHelper.h"
#include "constants.h"
#include <vector>
using namespace cv;

// [legacy]
// Roughly calculate color of an area inside detected circle
/*long long getAverageColor(cv::Mat& gray, cv::Point center) {
    long long av = 0;
    int sz = 2;
    for (int x = -sz; x != sz; ++x) {
        for (int y = -sz; y != sz; ++y) {
            av += gray.at<short>(center.x + x, center.y + y);
        }
    }
    return av / ((sz * 2 + 1) * (sz * 2 + 1));
}*/

// [legacy]
// Get vector of target image from the down side to the top in camera coords
std::pair<cv::Point, cv::Point> getNormVector(cv::Mat& gray, std::vector<cv::Vec3f> circles) {
    cv::Vec2f ret;
    std::vector<std::pair<long long, std::pair<int, int>>> colors;
    for (auto crc : circles) {
        cv::Point center(cvRound(crc[0]), cvRound(crc[1]));
        colors.push_back(std::make_pair(crc[2], std::make_pair(center.x, center.y)));
    }

    std::sort(colors.begin(), colors.end());
    cv::Point vecNormBegin = cv::Point(colors[0].second.first, colors[0].second.second);
    cv::Point vecNormEnd = cv::Point(colors[1].second.first, colors[1].second.second);

    return std::make_pair(vecNormBegin, vecNormEnd);
}


// Image processing function
void processImage(cv::Mat& src, CirclesMessage& msg, const struct ImageHandler& imgHandler) {
    Point2f camCenter(src.cols / 2, src.rows / 2);
    Mat threshold_output;
    cv::Mat notSmoothedGray;
    cv::Mat gray;
    cv::cvtColor(src, notSmoothedGray, CV_BGR2GRAY);
    std::vector<RotatedRect> minRect;
    std::vector<RotatedRect> minEllipse;
    // Smooth it, otherwise a lot of false circles may be detected
    cv::GaussianBlur(notSmoothedGray, gray, cv::Size(imgHandler.blurCoeff, imgHandler.blurCoeff), 2, 2);
    std::vector<cv::Vec3f> circles;

    const int minRadius = 10;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 21, 100, imgHandler.threshold, minRadius, imgHandler.maxRadius);
    bool foundFlag = true;
    Point2f targetPoint;

    // The following conditions can be changed to detect exactly 2 circles
    if (circles.size() < 1) {
        foundFlag = false;
    }
    if (foundFlag) {
        int targetX = 0;
        int targetY = 0;
        float averageRadius = 0.0;
        for(size_t i = 0; i < circles.size(); i++ )
        {
            averageRadius += circles[i][2];
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            targetX += cvRound(circles[i][0]);
            targetY += cvRound(circles[i][1]);
            int radius = cvRound(circles[i][2]);
            // draw the circle center
            cv::circle(src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
            // draw the circle outline
            cv::circle(src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
        }
        averageRadius /= circles.size();
        cv::circle(src, cv::Point(src.cols / 2, src.rows / 2) , averageRadius, cv::Scalar(255,255,0), 3, 8, 0 );
        // cv::arrowedLine(src, vc.first, vc.second, cv::Scalar(255, 0, 0), 4);

        targetPoint = Point2f(targetX / circles.size(), targetY / circles.size());
        // Find the rotated rectangles and ellipses for each contour
        minRect.push_back(RotatedRect(targetPoint, Size2f(averageRadius, averageRadius), 45));
        minEllipse.push_back(RotatedRect(targetPoint, Size2f(averageRadius, averageRadius), 45));
    }

    // Prepare the message
    msg.circles = minEllipse;
    msg.inTheBox = std::vector<bool>(minEllipse.size(), false);


    // Calculate central lines coordinates.

    Mat drawing = src;
    Point2f leftCenter(0, drawing.rows / 2), rightCenter(drawing.cols, drawing.rows / 2);
    Point2f topCenter(drawing.cols / 2, 0), bottomCenter(drawing.cols / 2, drawing.rows);

    // Here is a dynamic box implementation.
    // It changes own coords with the size of the circles.

    // Get circles size.
    float circleWidth = 50, circleHeight = 50;
    if (msg.circles.size() != 0) {
        circleWidth = msg.circles[0].size.width;
        circleHeight = msg.circles[0].size.height;
    }

    // Calculate box coordinates using linear function of circles size.
    float boxTop = std::max((float)1, drawing.rows * (-9 * circleHeight + 1900) / 4600),
          boxBottom = drawing.rows - boxTop,
          boxLeft = std::max((float)1, drawing.cols * (-9 * circleWidth + 2160) / 4600),
          boxRight = drawing.cols - boxLeft;

    Point2f boxTopLeft(boxLeft, boxTop), boxTopRight(boxRight, boxTop);
    Point2f boxBottomLeft(boxLeft, boxBottom), boxBottomRight(boxRight, boxBottom);

    // Prepare the message
    msg.box.left = boxLeft;
    msg.box.right = boxRight;
    msg.box.top = boxTop;
    msg.box.bottom = boxBottom;


    // Calculating rotation error in degrees
    float rotError = 0;
    if (circles.size() == 2) {
        std::pair<cv::Point, cv::Point> vecNorm = getNormVector(notSmoothedGray, circles);
        arrowedLine(drawing, vecNorm.first, vecNorm.second, Scalar(255, 0, 0), 2, CV_AA);
        cv::Point vecNorm2 = vecNorm.second - vecNorm.first;
        if (vecNorm2.y == 0) {
            if (vecNorm2.x < 0) {
                rotError = 90;
            } else {
                rotError = -90;
            }
        } else {
            rotError = atan2(vecNorm2.x, vecNorm2.y) * 180 / M_PI;
            rotError += 180;
            if (rotError > 180) {
                rotError -= 360;
            }
        }
        rotError /= 180; // Normalize error (max speed 1 = ~95 degrees/sec)
        msg.rotError = rotError;
    }


    if (foundFlag &&
            (targetPoint.x > boxRight ||
             targetPoint.x < boxLeft ||
             targetPoint.y < boxTop ||
             targetPoint.y > boxBottom) )
    {
        arrowedLine(drawing, camCenter, targetPoint, Scalar(255, 0, 0), 2, CV_AA);
        msg.inTheBox[0] = true;
    }

    // Draw central lines, box and circle
    line( drawing, leftCenter, rightCenter, Scalar(255, 0, 0), 3, 8 );
    line( drawing, bottomCenter, topCenter, Scalar(255, 0, 0), 3, 8 );
    rectangle( drawing, boxBottomLeft, boxTopRight, Scalar(100, 0, 255, 0.1), 2, CV_AA);
}

