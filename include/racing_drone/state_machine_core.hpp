/**
 * @file state_machine.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief System State Machine for autonomous drone 
 * @version 0.1
 * @date 01-18-2020
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
#include <ros/time.h>
#include <racing_drone/DroneState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

enum STATE_CHANGE_TYPE
{
    DISTANCE=0,
    TIME
};


enum STATE_TYPE
{
    GATE=0,
    WAYPOINT, 
};

class State
{
    public:
    int id;
    int next_state_id;
    racing_drone::DroneState des_state;
    STATE_TYPE state_type;
    STATE_CHANGE_TYPE state_change_type;
    double change_threshold;

    State(int id_, int nxt_id_, racing_drone::DroneState state_, STATE_TYPE st_typ, STATE_CHANGE_TYPE st_ch_typ, double thshld_);

};

class StateMachine
{
    public:
    ros::NodeHandle nh;
    ros::Publisher controlRefPub;
    ros::Publisher nextGatePub;
    ros::Subscriber odomSub;
    ros::Subscriber autonomySub;

    std::string controlRefPubTopic;
    std::string odomSubTopic;
    std::string autonomySubTopic;


    std::vector<State> states;
    State state;
    int curr_state_id;
    int next_gate_id;

    racing_drone::DroneState curr_drone_state;
    ros::Time start_time;
    ros::Time start_lap_time;
    double time_elapsed;
    double lap_time;
    double state_error; 

    bool autonomy;

    StateMachine(ros::NodeHandle& nh_, std::string controlRefPubTopic_, std::string odomSubTopic_, 
                                std::string autonomySubTopic_, std::vector<State> states_ );

    void updateState(void);
    void updateStateError(void);
    void findNextGate(void);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void autonomyCallback(const std_msgs::Bool::ConstPtr& atnmy);
};
