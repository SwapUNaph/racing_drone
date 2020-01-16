/**
 * @file camera.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief ROS node for camera, publishes RGB image
 * @version 0.1
 * @date 01-11-2020
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

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class Camera
{
    public:
    cv::VideoCapture cap;
    cv_bridge::CvImage cvImg;
    int frame_width;
    int frame_height;

    Camera()
    {
        //Camera Video Capture Properties
        cap = cv::VideoCapture(0);
        while(!cap.isOpened());  // poll till cap is opened
        //720p : 2560x720 : 60 FPS
        //WVGA : 1344x376 : 100 FPS
        cap.set(5, 60);	// FPS
        cap.set(3, 2560); //Image Width
        cap.set(4, 720);	//Image Height
        // cap.set(12, 0.5); //Saturation
        // cap.set(10, 0.5); //Brightness
        frame_width = (int)(cap.get(3)/2);
        frame_height = (int)(cap.get(4));  
        cvImg.encoding = "bgr8";
        std::cout << "Frame height: " << frame_height << ", Frame width: " << frame_width << std::endl;
    }

    ~Camera()
    {
        cap.release();
    }


    int captureImage()
    {
        //Get the image
        cap >> cvImg.image;
        if(cvImg.image.empty())
            return 0;

        cvImg.image = cvImg.image(cv::Rect(0,0, frame_width, frame_height));
        return 1;
    }


};

int main(int argc, char **argv)
{  
  // Set up ROS.
  ros::init(argc, argv, "state_estimator");
  
  std::string imageTopic = "/camera/image_raw";

  ros::NodeHandle nh("");

  ros::Publisher imPub = nh.advertise<sensor_msgs::Image>(imageTopic, 1);

  Camera camera;

  // Main loop.
  while (1)
  {
      if( !camera.captureImage() )
        continue;
    
      imPub.publish(camera.cvImg.toImageMsg());
      
  }

}