
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
  vector<int> hsv_low_thresh;
  vector<int> hsv_high_thresh;
  int blur_kernel;
  double area_thresh;
  double aspect_ratio_low;
  double roi_mean_thresh;
  vector<double> cameraMatrix;
  vector<double> distCoeffs;
  std::string pubTopic;

  ros::param::get("gate_detector/gateSide", gateSide);
  ros::param::get("gate_detector/hsv_low_thresh", hsv_low_thresh);
  ros::param::get("gate_detector/hsv_high_thresh", hsv_high_thresh);
  ros::param::get("gate_detector/blur_kernel", blur_kernel);
  ros::param::get("gate_detector/area_thresh", area_thresh);
  ros::param::get("gate_detector/aspect_ratio_low", aspect_ratio_low);
  ros::param::get("gate_detector/roi_mean_thresh", roi_mean_thresh);
  ros::param::get("gate_detector/cameraMatrix", cameraMatrix);
  ros::param::get("gate_detector/distCoeffs", distCoeffs);
  ros::param::get("gate_detector/pubTopic", pubTopic);

  ROS_INFO("[Gate_detector] gateSide: %f m", gateSide);
  ROS_INFO("[Gate_detector] hsv_low_thresh: %d, %d, %d", hsv_low_thresh[0], hsv_low_thresh[1], hsv_low_thresh[2] );
  ROS_INFO("[Gate_detector] hsv_high_thresh: %d, %d, %d", hsv_high_thresh[0], hsv_high_thresh[1], hsv_high_thresh[2] );
  ROS_INFO("[Gate_detector] blur_kernel: %d", blur_kernel);
  ROS_INFO("[Gate_detector] area_thresh: %f", area_thresh);
  ROS_INFO("[Gate_detector] aspect_ratio_low: %f", aspect_ratio_low);
  ROS_INFO("[Gate_detector] roi_mean_thresh: %f", roi_mean_thresh);
  ROS_INFO("[Gate_detector] pubTopic: %s", pubTopic.c_str());
  ROS_INFO("[Gate_detector] cameraMatrix[0:3]: %f, %f, %f", cameraMatrix[0], cameraMatrix[1], cameraMatrix[2]);
  ROS_INFO("[Gate_detector] distCoeffs[0:3]: %f, %f, %f", distCoeffs[0], distCoeffs[1], distCoeffs[2]);

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

  GateDetectorCore gateDetectorNode(nh, gd, pubTopic);

  ros::Rate gdRate(100); // Tell ROS how fast to run this node.

  //Wait for other nodes to initialize
  ros::Rate sleepRate(0.2);
	sleepRate.sleep();

  //Camera Video Capture Properties
  VideoCapture cap(0);
  while(!cap.isOpened());  // poll till cap is opened
	//720p : 2560x720 : 60 FPS
	//WVGA : 1344x376 : 100 FPS
	cap.set(5, 30);	// FPS
	cap.set(3, 640); //Image Width
	cap.set(4, 480);	//Image Height
	cap.set(12, 0.5); //Saturation
	cap.set(10, 0.5); //Brightness
	int frame_width = (int)(cap.get(3) / 2);
	int frame_height = (int)(cap.get(4));  
  cout << "Frame height: " << frame_height << ", Frame width: " << frame_width << endl;

  Mat image; //Image to capture
  // Main loop.
  while (gateDetectorNode.nh.ok())
  {
    //Get the image
    cap >> image;
    if(image.empty())
      continue;
	  image = image(Rect(0,0, frame_width, frame_height));

    ros::spinOnce();
    ros::Time begin = ros::Time::now();
    gateDetectorNode.publishGatePose(image);
    ros::Time end = ros::Time::now();
	  double loopTime = end.toNSec() - begin.toNSec();
    ROS_INFO("\nGate detector loop time: %f ms\n", loopTime/1e6);
    gdRate.sleep();
  }

  return 0;
} // end main()
