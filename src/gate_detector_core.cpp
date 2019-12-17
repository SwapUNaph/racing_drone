#include "racing_drone/gate_detector_core.hpp"

GateDetectorCore::GateDetectorCore(ros::NodeHandle &node_handle, GateDetector gd_, std::string pubTopic_) 
                            :   nh(node_handle) , gd(gd_), pubTopic(pubTopic_)
{
     gatePosePub = nh.advertise<geometry_msgs::Pose>(pubTopic, 5);
}

GateDetectorCore::~GateDetectorCore(){}

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


