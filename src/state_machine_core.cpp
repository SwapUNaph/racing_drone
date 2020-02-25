/**
 * @file state_machine_core.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief State Machine
 * @version 0.1
 * @date 01-19-2020
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

#include <racing_drone/state_machine_core.hpp>
#include "common.cpp"

State::State(int id_, int nxt_id_, racing_drone::DroneState state_, STATE_TYPE st_typ, STATE_CHANGE_TYPE st_ch_typ, double thshld_)
    : id(id_), next_state_id(nxt_id_), des_state(state_), state_type(st_typ), state_change_type(st_ch_typ), change_threshold(thshld_)
{

}

StateMachine::StateMachine(ros::NodeHandle& nh_, std::string controlRefPubTopic_, std::string odomSubTopic_, 
                                std::string autonomySubTopic_, std::vector<State> states_ )
                        : nh(nh_), controlRefPubTopic(controlRefPubTopic_), odomSubTopic(odomSubTopic_), 
                            autonomySubTopic(autonomySubTopic_), states(states_), state(states_[0])
{
    controlRefPub = nh.advertise<racing_drone::DroneState>(controlRefPubTopic, 5);
    nextGatePub = nh.advertise<racing_drone::DroneState>("/state_machine/next_gate", 5);
    odomSub = nh.subscribe(odomSubTopic, 5, &StateMachine::odomCallback, this);
    autonomySub = nh.subscribe(autonomySubTopic, 5, &StateMachine::autonomyCallback, this);

    start_time = ros::Time::now();
    time_elapsed = 0;
    lap_time = 0;
    curr_drone_state = racing_drone::DroneState();
    curr_state_id = 0;
    next_gate_id = 1;

    autonomy = true;
}

void StateMachine::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    curr_drone_state.position.x = odom->pose.pose.position.x;
    curr_drone_state.position.y = odom->pose.pose.position.y;
    curr_drone_state.position.z = odom->pose.pose.position.z;
    curr_drone_state.velocity.x = odom->twist.twist.linear.x;
    curr_drone_state.velocity.y = odom->twist.twist.linear.y;
    curr_drone_state.velocity.z = odom->twist.twist.linear.z;

    std::vector<double> quat(4), rpy(4);
    quat[0] = odom->pose.pose.orientation.x;
    quat[1] = odom->pose.pose.orientation.y;
    quat[2] = odom->pose.pose.orientation.z;
    quat[3] = odom->pose.pose.orientation.w;
    quat2rpy(quat, rpy);

    curr_drone_state.yaw = rpy[2];

    updateState();

}

void StateMachine::autonomyCallback(const std_msgs::Bool::ConstPtr& atnmy)
{
    autonomy = atnmy->data;
}

void StateMachine::updateState(void)
{
    ros::Time time_now = ros::Time::now();
    time_elapsed = (time_now.toNSec() - start_time.toNSec()) / 1e9 ;
    lap_time = (time_now.toNSec() - start_lap_time.toNSec()) / 1e9 ;

    updateStateError();
    findNextGate();
    nextGatePub.publish(states[next_gate_id].des_state);

    bool STATE_CHANGE = false;

    if( state.state_change_type == STATE_CHANGE_TYPE::DISTANCE )
        if( state_error < state.change_threshold || time_elapsed > 5.0)
            STATE_CHANGE = true;

    if( state.state_change_type == STATE_CHANGE_TYPE::TIME )
        if( time_elapsed > state.change_threshold )
            STATE_CHANGE = true;


    if( state.id == 1 && time_elapsed < 0.1)
    {
        lap_time = 0.0;
        start_lap_time = ros::Time::now();
    }

    if( autonomy )
    {
        // enable controller
        controlRefPub.publish(state.des_state);
    }
    else
    {
        // disable controller
        if( STATE_CHANGE )
            STATE_CHANGE = false;
    }

    if ( STATE_CHANGE )
    {
        state = states[state.next_state_id];
        curr_state_id = state.id;
        start_time = ros::Time::now();
        time_elapsed = 0.0;
    }



}

void StateMachine::updateStateError(void)
{
    double dist_error = sqrt( pow(curr_drone_state.position.x - state.des_state.position.x, 2) +
                              pow(curr_drone_state.position.y - state.des_state.position.y, 2) +
                              pow(curr_drone_state.position.z - state.des_state.position.z, 2) );

    double vel_error = sqrt( pow(curr_drone_state.velocity.x - state.des_state.velocity.x, 2) +
                             pow(curr_drone_state.velocity.y - state.des_state.velocity.y, 2) +
                             pow(curr_drone_state.velocity.z - state.des_state.velocity.z, 2) );


    double yaw_error = std::abs(curr_drone_state.yaw - state.des_state.yaw);

    state_error =  dist_error; //+ 2.0 * yaw_error;

    ROS_INFO( "Current State id: %d, Current_Gate_id: %d, State Error: %.2f, Time Elapsed: %.3f, Autonomy: %d", state.id, next_gate_id, state_error, time_elapsed, autonomy );
    ROS_INFO( "Lap Time: %.3f s", lap_time);
}

void StateMachine::findNextGate(void)
{
    for(int i=curr_state_id; i<states.size(); i++)
    {
        if( states[i].state_type == STATE_TYPE::GATE)
        {
            next_gate_id = i;
            break;
        }

        if(i == states.size()-1)
            i = 0;
    }

}