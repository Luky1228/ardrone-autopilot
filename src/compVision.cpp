
/*  
    compVision.cpp

  -------------------------------------------------------------------
  | Provides the computer vision algorithm that processes           |     
  |  the image received from the imageHandler.cpp.                  |
  |                                                                 |
  | Sends back extracted information about the circles and the box. |
  |                                                                 |
  | Algorithm realisation is based on OpenCV tutorial               |
  |  from the documentation - goo.gl/rGCvhv.                        |
  -------------------------------------------------------------------

*/

#include <algorithm>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs.hpp>
#include "imgHelper.h"
using namespace cv;

double getAverageColor(cv::Mat& gray, cv::Point center) {
    double av = 0;
    int sz = 2;
    for (int x = -sz; x != sz; ++x)
    {
        for (int y = -sz; y != sz; ++y)
        {
            av += gray.at<int>(center.x + x, center.y + y);
        }
    }
    return av / (sz * sz);
}

std::pair<cv::Point, cv::Point> getTargetVector(cv::Mat& gray, std::vector<cv::Vec3f> circles) {
    cv::Vec2f ret;
    std::vector<std::pair<double, std::pair<int, int>>> colors;
    for (auto crc : circles)
    {
        cv::Point center(cvRound(crc[0]), cvRound(crc[1]));
        colors.push_back(std::make_pair(getAverageColor(gray, center), std::make_pair(center.x, center.y)));
    }
/*
    std::sort(colors.begin(), colors.end());
    std::reverse(colors.begin(), colors.end());
*/

/*

    cv::Point vecHead = cv::Point(colors[0].second.first, colors[0].second.second);
    cv::Point vecTail = cv::Point((colors[1].second.first + colors[2].second.first) / 2,
        (colors[1].second.second + colors[2].second.second) / 2);
    return std::make_pair(vecTail, vecHead);
*/

    cv::Point vecHead = cv::Point(colors[0].second.first, colors[0].second.second);
    cv::Point vecTail = cv::Point((colors[0].second.first + colors[1].second.first + colors[2].second.first) / 3,
        (colors[0].second.second + colors[1].second.second + colors[2].second.second) / 3);
    return std::make_pair(vecTail, vecHead);
}


// Image processing function.

void processImage(cv::Mat& src, CirclesMessage& msg) {
    Point2f camCenter(src.cols / 2, src.rows / 2);
    Mat threshold_output;
    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    std::vector<RotatedRect> minRect;
    std::vector<RotatedRect> minEllipse;
    // smooth it, otherwise a lot of false circles may be detected
    cv::GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 90);
    bool foundFlag = true;
    Point2f targetPoint;
    if (circles.size() != 3)
    { 
        foundFlag = false;
    }
    if (foundFlag)
    {
        for(size_t i = 0; i < circles.size(); i++ ) 
        {
             cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
             int radius = cvRound(circles[i][2]);
             // draw the circle center
             cv::circle(src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
             // draw the circle outline
             cv::circle(src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
        }
        auto vc = getTargetVector(gray, circles);
 //       cv::arrowedLine(src, vc.first, vc.second, cv::Scalar(255, 0, 0), 4);
        targetPoint = vc.first;
      /// Find the rotated rectangles and ellipses for each contour 
        minRect.push_back(RotatedRect(vc.first, Size2f(50,50), 45));
        minEllipse.push_back(RotatedRect(vc.first, Size2f(50,50), 45));
    }

  // Preparing the message

  msg.circles = minEllipse;
  msg.inTheBox = std::vector<bool>(minEllipse.size(), false);

  Mat drawing = src;



  // Calculate central lines coordinates.

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
    
    // Calculation of the box coordinates using linear function of circles size.
    
       float boxTop = std::max((float)1, drawing.rows * (-9 * circleHeight + 1900) / 4600),
	         boxBottom = drawing.rows - boxTop,
	         boxLeft = std::max((float)1, drawing.cols * (-9 * circleWidth + 2160) / 4600),
	         boxRight = drawing.cols - boxLeft;
       
       Point2f boxTopLeft(boxLeft, boxTop), boxTopRight(boxRight, boxTop);
       Point2f boxBottomLeft(boxLeft, boxBottom), boxBottomRight(boxRight, boxBottom);
    
    // Preparing the message.

       msg.box.left = boxLeft;
       msg.box.right = boxRight;
       msg.box.top = boxTop;
       msg.box.bottom = boxBottom;
       
       if (foundFlag && 
           (targetPoint.x > boxRight ||
           targetPoint.x < boxLeft ||
           targetPoint.y < boxTop ||
           targetPoint.y > boxBottom) )
       {
           arrowedLine(drawing, camCenter, targetPoint, Scalar(255, 0, 0), 2, CV_AA);
           msg.inTheBox[0] = false;
       }

      //Draw central lines and the box.

       line( drawing, leftCenter, rightCenter, Scalar(255, 0, 0), 3, 8 );
       line( drawing, bottomCenter, topCenter, Scalar(255, 0, 0), 3, 8 );
       rectangle( drawing, boxBottomLeft, boxTopRight, Scalar(100, 0, 255, 0.1), 2, CV_AA);
}

