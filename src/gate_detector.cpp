
/**
 * @file gate_detector.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief gate_detector ROS Node
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

#include "racing_drone/gate_detector_core.hpp"
#include <opencv2/video/video.hpp>


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "gate_detector");
  ros::NodeHandle nh("");

  //Load parameters
  double gateSide;
  std::vector<int> hsv_low_thresh;
  std::vector<int> hsv_high_thresh;
  int blur_kernel;
  double area_thresh;
  double aspect_ratio_low;
  double roi_mean_thresh;
  std::vector<double> cameraMatrix;
  std::vector<double> distCoeffs;
  std::vector<double> mnVar;
  std::vector<double> pnVar;
  double EKFrate;
  std::string rawGatePubTopic;
  std::string filteredGatePubTopic;
  std::string odomSubTopic;
  std::string imageTopic;
  std::string imuTopic;

  ros::param::get("gate_detector/gateSide", gateSide);
  ros::param::get("gate_detector/hsv_low_thresh", hsv_low_thresh);
  ros::param::get("gate_detector/hsv_high_thresh", hsv_high_thresh);
  ros::param::get("gate_detector/blur_kernel", blur_kernel);
  ros::param::get("gate_detector/area_thresh", area_thresh);
  ros::param::get("gate_detector/aspect_ratio_low", aspect_ratio_low);
  ros::param::get("gate_detector/roi_mean_thresh", roi_mean_thresh);
  ros::param::get("gate_detector/cameraMatrix", cameraMatrix);
  ros::param::get("gate_detector/distCoeffs", distCoeffs);
  ros::param::get("gate_detector/meas_noise_variance", mnVar);
  ros::param::get("gate_detector/process_noise_variance", pnVar);
  ros::param::get("gate_detector/EKFrate", EKFrate);
  ros::param::get("gate_detector/rawGatePubTopic", rawGatePubTopic);
  ros::param::get("gate_detector/filteredGatePubTopic", filteredGatePubTopic);
  ros::param::get("gate_detector/odomSubTopic", odomSubTopic);
  ros::param::get("gate_detector/imageTopic", imageTopic);
  ros::param::get("gate_detector/imuTopic", imuTopic);


  // ROS_INFO("[Gate_detector] gateSide: %f m", gateSide);
  // ROS_INFO("[Gate_detector] hsv_low_thresh: %d, %d, %d", hsv_low_thresh[0], hsv_low_thresh[1], hsv_low_thresh[2] );
  // ROS_INFO("[Gate_detector] hsv_high_thresh: %d, %d, %d", hsv_high_thresh[0], hsv_high_thresh[1], hsv_high_thresh[2] );
  // ROS_INFO("[Gate_detector] blur_kernel: %d", blur_kernel);
  // ROS_INFO("[Gate_detector] area_thresh: %f", area_thresh);
  // ROS_INFO("[Gate_detector] aspect_ratio_low: %f", aspect_ratio_low);
  // ROS_INFO("[Gate_detector] roi_mean_thresh: %f", roi_mean_thresh);
  // ROS_INFO("[Gate_detector] pubTopic: %s", pubTopic.c_str());
  // ROS_INFO("[Gate_detector] cameraMatrix[0:3]: %f, %f, %f", cameraMatrix[0], cameraMatrix[1], cameraMatrix[2]);
  // ROS_INFO("[Gate_detector] distCoeffs[0:3]: %f, %f, %f", distCoeffs[0], distCoeffs[1], distCoeffs[2]);

  Scalar hsv_l_th(hsv_low_thresh[0], hsv_low_thresh[1], hsv_low_thresh[2]);
  Scalar hsv_h_th(hsv_high_thresh[0], hsv_high_thresh[1], hsv_high_thresh[2]);
  Mat camMat = Mat(cameraMatrix, true).reshape(3,3);
  Mat distCoef = Mat(distCoeffs, true).reshape(1,5);

  GateDetector gd(  gateSide,
                    hsv_l_th,
                    hsv_h_th,
                    blur_kernel,
                    area_thresh,
                    aspect_ratio_low,
                    roi_mean_thresh,
                    camMat,
                    distCoef   );

  //EKF Instantiation
    ublas::matrix<double> a(3,3);
    ublas::matrix<double> b(3,3);
    ublas::matrix<double> h(3,3);
    ublas::matrix<double> q(3,3);
    ublas::matrix<double> r(3,3);

    for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
          a(i,j) = 0.0;

    b(0,0) = -1.0;  b(0,1) = 0.0;  b(0,2) = 0.0;     
    b(1,0) = 0.0;  b(1,1) = -1.0;  b(1,2) = 0.0;     
    b(2,0) = 0.0;  b(2,1) = 0.0;  b(2,2) = -1.0;     
    // b(3,0) = 1.0;  b(3,1) = 0.0;  b(3,2) = 0.0;     
    // b(4,0) = 0.0;  b(4,1) = 1.0;  b(4,2) = 0.0;     
    // b(5,0) = 0.0;  b(5,1) = 0.0;  b(5,2) = 1.0;     

    for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
    {
        if(i == j)
          h(i,j) = 1.0;
        else
          h(i,j) = 0.0;
        
    }

    for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
    {
        if(i == j)
          q(i,j) = pnVar[i];
        else
          q(i,j) = 0.0;
        
    }

    for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
    {
        if(i == j)
          r(i,j) = mnVar[i];
        else
          r(i,j) = 0.0;
        
    }

  ExtendedKalmanFilter gEKF(a,b,h,q,r, 1.0 / EKFrate);

  GateDetectorCore gateDetectorNode(nh, gd, gEKF, rawGatePubTopic, filteredGatePubTopic, odomSubTopic, imageTopic, imuTopic);

  //Wait for other nodes to initialize
  ros::Rate sleepRate(1.0);
	sleepRate.sleep();


  //Multithreaded ROS spinner
  ros::MultiThreadedSpinner multi(6);
	ROS_INFO("Gate-detector initiated...");
  ros::spin(multi);

  return 0;
} // end main()
