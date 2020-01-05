/**
 * @file gate_detector_core.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief gate_detector_core definition
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

/**
 * @brief Construct a new Gate Detector Core:: Gate Detector Core object
 * 
 * @param node_handle ROS Node handle 
 * @param gd_ GateDetector object
 * @param pubTopic_ Gate pose publication topic
 */
GateDetectorCore::GateDetectorCore(ros::NodeHandle &node_handle, GateDetector gd_, std::string pubTopic_) 
                            :   nh(node_handle) , gd(gd_), pubTopic(pubTopic_)
{
     gatePosePub = nh.advertise<geometry_msgs::Pose>(pubTopic, 5);
}

/**
 * @brief Destroy the Gate Detector Core:: Gate Detector Core object
 * 
 */
GateDetectorCore::~GateDetectorCore(){}

/**
 * @brief Detect Gate, Calcualte Gate Pose and publish Gate Pose
 * 
 * @param img Input image for gate detection
 */
void GateDetectorCore::publishGatePose(Mat& img)
{
    gatePose = geometry_msgs::Pose();

    if( gd.detectGate(img) )
    {
        if( gd.getGatePose() )
        {
            gatePose.position.x = gd.tvec[0];
            gatePose.position.y = gd.tvec[1];
            gatePose.position.z = gd.tvec[2];

            vector<double> gateQ = rvec2quat(gd.rvec);
            gatePose.orientation.x = gateQ[0];
            gatePose.orientation.y = gateQ[1];
            gatePose.orientation.z = gateQ[2];
            gatePose.orientation.w = gateQ[3];
        }
    }

    gatePosePub.publish(gatePose);
    
}


