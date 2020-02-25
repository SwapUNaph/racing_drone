/**
 * @file GateDetector.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief GateDetector class declaration
 * @version 0.1
 * @date 01-05-2020
 * 
 *  Copyright (c) 2020 Swapneel Naphade
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <ctime>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <iostream>



using namespace cv;
using namespace std;

double roiMean(std::vector<Point> contour, Mat image);
bool compareContourArea(std::vector<Point> a, std::vector<Point> b);
double aspectRatio(const std::vector<Point>& contour);
std::vector<double> rvec2quat(std::vector<double> rvec);
std::vector<double> quat2euler(std::vector<double> q);


class GateDetector
{
    public:
    double gateSide;
    Scalar hsv_low_thresh;
    Scalar hsv_high_thresh;
    int blur_kernel;
    double area_thresh;
    double aspect_ratio_low;
    double roi_mean_thresh;
    double gate_dist_thresh;
    Mat cameraMatrix;
    Mat distCoeffs;


    std::vector<Point> gateContour;
    std::vector<std::vector<Point>> detectedContours;
    std::vector<double> tvec;
    std::vector<double> rvec;

    GateDetector(   double gateSide_,
                    Scalar hsv_low_thresh_,
                    Scalar hsv_high_thresh_,
                    int blur_kernel_,
                    double area_thresh_,
                    double aspect_ratio_low_,
                    double roi_mean_thresh_,
                    double gate_dist_thresh,
                    Mat cameraMatrix_,
                    Mat distCoeffs_  );

    ~GateDetector();
    bool detectGate(Mat& image);
    bool getGatePose(void);

};