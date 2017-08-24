#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

const int circleRadius = 50;

double getAverageColor(cv::Mat& gray, cv::Point center) {
    double av = 0;
    int sz = 2;
    for (int x = -sz; x != sz; ++x)
        for (int y = -sz; y != sz; ++y)
            av += gray.at<int>(center.x + x, center.y + y);
    return av / (sz * sz);
}

std::pair<cv::Point, cv::Point> getTargetVector(cv::Mat& gray, std::vector<cv::Vec3f> circles) {
    cv::Vec2f ret;
    std::vector<std::pair<double, std::pair<int, int>>> colors;
    for (auto crc : circles) {
        cv::Point center(cvRound(crc[0]), cvRound(crc[1]));
        colors.push_back(std::make_pair(getAverageColor(gray, center), std::make_pair(center.x, center.y)));
    }
    std::sort(colors.begin(), colors.end());
    std::reverse(colors.begin(), colors.end());

    cv::Point vecHead = cv::Point(colors[0].second.first, colors[0].second.second);
    cv::Point vecTail = cv::Point((colors[1].second.first + colors[2].second.first) / 2,
        (colors[1].second.second + colors[2].second.second) / 2);
    return std::make_pair(vecTail, vecHead);
}

void processImage(cv::Mat& img) {
    cv::Mat gray;
    //img = cv::imread("../src/circles-new3.jpg");
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    cv::GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 90);
    if (circles.size() != 3)
        return;
    for(size_t i = 0; i < circles.size(); i++ ) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         cv::circle(img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         cv::circle(img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

    }
    auto vc = getTargetVector(gray, circles);
    cv::arrowedLine(img, vc.first, vc.second, cv::Scalar(255, 0, 0), 4);
    std::cout << "found  " << circles.size() << " circles";
    return;
}
