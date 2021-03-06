/**
 * @file state_machine.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief State Machine
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



#include <racing_drone/state_machine_core.hpp>

#define PI 3.141592653589793

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle nh;

    // ros::Publisher takeoffPub;
    // takeoffPub = nh.advertise<std_msgs::Bool>("/ardrone/takeoff", 5);

    // Wait for initialization
    ros::Rate rate(0.5);
    rate.sleep();

    std::vector<double> states_str;
    std::string odom_sub_topic;

    ros::param::get("trajectory/states", states_str);
    ros::param::get("controller/odom_sub_topic", odom_sub_topic);

    int N = 10;
    int Ns = states_str.size()/N;

    std::vector<State> states;

    for(int i=0; i < Ns; i++ )
    {
        racing_drone::DroneState pointState;
        pointState.position.x = states_str[10*i + 0];
        pointState.position.y = states_str[10*i + 1];
        pointState.position.z = states_str[10*i + 2];
        pointState.velocity.x = states_str[10*i + 3];
        pointState.velocity.y = states_str[10*i + 4];
        pointState.velocity.z = states_str[10*i + 5];
        pointState.yaw        = states_str[10*i + 6];
        STATE_TYPE st_typ = (states_str[10*i + 7] == 1.0) ? (STATE_TYPE::WAYPOINT) : (STATE_TYPE::GATE);
        STATE_CHANGE_TYPE st_chg_typ = (states_str[10*i + 8] == 1.0) ? (STATE_CHANGE_TYPE::DISTANCE) : (STATE_CHANGE_TYPE::TIME);
        double st_cng_thresh = states_str[10*i + 9];

        int next_state_id = (i < Ns-1) ? i+1 : 1;
        states.push_back(State(i, next_state_id, pointState, st_typ, st_chg_typ, st_cng_thresh));
    }
  

   

    // Setup State Machine
    // std::vector<racing_drone::DroneState> des_states(50);
    // des_states[0].position.x = 0.0;     des_states[0].position.y = 0.0;     des_states[0].position.z = 2.25;         des_states[0].velocity.x = 0.0;      des_states[0].velocity.y = 0.0;         des_states[0].velocity.z = 0.0;         des_states[0].yaw = -PI/2.0;

    // Two gates arena
    // des_states[1].position.x = 4.0;     des_states[1].position.y = -0.75;     des_states[1].position.z = 2.25;        des_states[1].velocity.x = 5.0;      des_states[1].velocity.y = 0.0;         des_states[1].velocity.z = 0.0;         des_states[1].yaw = 0.0;
    // des_states[2].position.x = 7.5;     des_states[2].position.y = -0.75;     des_states[2].position.z = 2.25;         des_states[2].velocity.x = 2.0;      des_states[2].velocity.y = 0.0;         des_states[2].velocity.z = 0.0;         des_states[2].yaw = 0.0;
    // des_states[3].position.x = 8.5;     des_states[3].position.y = -0.75;     des_states[3].position.z = 2.25;         des_states[3].velocity.x = 0.0;      des_states[3].velocity.y = 0.0;         des_states[3].velocity.z = 0.0;         des_states[3].yaw = PI ;
    // des_states[4].position.x = 8.5;     des_states[4].position.y = 1.5;     des_states[4].position.z = 2.25;         des_states[4].velocity.x = 0.0;      des_states[4].velocity.y = 0.0;         des_states[4].velocity.z = 0.0;         des_states[4].yaw = PI ;
    // des_states[5].position.x = 4.0;     des_states[5].position.y = 0.75;     des_states[5].position.z = 2.25;         des_states[5].velocity.x = -5.0;      des_states[5].velocity.y = 0.0;         des_states[5].velocity.z = 0.0;         des_states[5].yaw = PI ;
    // des_states[6].position.x = -4.0;     des_states[6].position.y = 0.75;     des_states[6].position.z = 2.25;         des_states[6].velocity.x = -5.0;      des_states[6].velocity.y = 0.0;         des_states[6].velocity.z = 0.0;         des_states[6].yaw = PI ;
    // des_states[7].position.x = -7.5;     des_states[7].position.y = 0.75;     des_states[7].position.z = 2.25;         des_states[7].velocity.x = -2.0;      des_states[7].velocity.y = 0.0;         des_states[7].velocity.z = 0.0;         des_states[7].yaw =  PI ; 
    // des_states[8].position.x = -8.5;     des_states[8].position.y = 0.75;     des_states[8].position.z = 2.25;         des_states[8].velocity.x = 0.0;      des_states[8].velocity.y = 0.0;         des_states[8].velocity.z = 0.0;         des_states[8].yaw = 0.0;
    // des_states[9].position.x = -8.5;     des_states[9].position.y = -1.5;     des_states[9].position.z = 2.25;         des_states[9].velocity.x = 2.0;      des_states[9].velocity.y = 0.0;         des_states[9].velocity.z = 0.0;         des_states[9].yaw = 0.0 ;
    // des_states[10].position.x = -4.0;     des_states[10].position.y = -0.75;     des_states[10].position.z = 2.25;         des_states[10].velocity.x = 5.0;      des_states[10].velocity.y = 0.0;         des_states[10].velocity.z = 0.0;         des_states[10].yaw = 0.0;                       
    // des_states[10].position.x = 0.0;     des_states[10].position.y = 0.0;     des_states[10].position.z = 2.25;         des_states[10].velocity.x = 0.0;      des_states[10].velocity.y = 0.0;         des_states[10].velocity.z = 0.0;         des_states[10].yaw = 0.0;             
    // des_states[11].position.x = 0.0;     des_states[11].position.y = 0.0;     des_states[11].position.z = 2.25;         des_states[11].velocity.x = 0.0;      des_states[11].velocity.y = 0.0;         des_states[11].velocity.z = 0.0;         des_states[11].yaw = 0.0;   

    // Four gates arena
    // des_states[1].position.x = 9.0;     des_states[1].position.y = 0.0;      des_states[1].yaw = 0.0;            des_states[1].position.z = 2.25;         des_states[1].velocity.x = 2.0;      des_states[1].velocity.y = 0.0;         des_states[1].velocity.z = 0.0;   
    // des_states[2].position.x = 11.0;     des_states[2].position.y = 0.0;      des_states[2].yaw = 0.0;            des_states[2].position.z = 2.25;         des_states[2].velocity.x = 0.0;      des_states[2].velocity.y = 0.0;         des_states[2].velocity.z = 0.0;   
    // des_states[3].position.x = 10.0;     des_states[3].position.y = -9.0;      des_states[3].yaw = - PI / 2.0;     des_states[3].position.z = 2.25;         des_states[3].velocity.x = 0.0;      des_states[3].velocity.y = -2.0;         des_states[3].velocity.z = 0.0;   
    // des_states[4].position.x = 10.0;     des_states[4].position.y = -11.0;      des_states[4].yaw = - PI / 2.0;     des_states[4].position.z = 2.25;         des_states[4].velocity.x = 0.0;      des_states[4].velocity.y = 0.0;         des_states[4].velocity.z = 0.0;   
    // des_states[5].position.x = 1.0;     des_states[5].position.y = -10.0;      des_states[5].yaw = - PI;            des_states[5].position.z = 2.25;         des_states[5].velocity.x = 0.0;      des_states[5].velocity.y = 0.0;         des_states[5].velocity.z = 0.0;   
    // des_states[6].position.x = -1.0;     des_states[6].position.y = -10.0;      des_states[6].yaw = - PI;            des_states[6].position.z = 2.25;         des_states[6].velocity.x = 0.0;      des_states[6].velocity.y = 0.0;         des_states[6].velocity.z = 0.0;   
    // des_states[7].position.x = 0.0;     des_states[7].position.y = -2.0;      des_states[7].yaw = PI / 2.0 ;         des_states[7].position.z = 2.25;         des_states[7].velocity.x = 0.0;      des_states[7].velocity.y = 2.0;         des_states[7].velocity.z = 0.0;   
    // des_states[8].position.x = 0.0;     des_states[8].position.y = 0.0;      des_states[8].yaw = PI / 2.0 ;         des_states[8].position.z = 2.25;         des_states[8].velocity.x = 0.0;      des_states[8].velocity.y = 0.0;         des_states[8].velocity.z = 0.0;   
   
    // std::vector<State> states{ State(0, 1, des_states[0], STATE_TYPE::WAYPOINT, STATE_CHANGE_TYPE::TIME, 2.0) ,
    //                            State(1, 2, des_states[1], STATE_TYPE::GATE, STATE_CHANGE_TYPE::DISTANCE, 1.0) ,
    //                            State(2, 3, des_states[2], STATE_TYPE::WAYPOINT, STATE_CHANGE_TYPE::DISTANCE, 0.7) ,
    //                            State(3, 4, des_states[3], STATE_TYPE::GATE, STATE_CHANGE_TYPE::DISTANCE, 1.0) ,
    //                            State(4, 5, des_states[4], STATE_TYPE::WAYPOINT, STATE_CHANGE_TYPE::DISTANCE, 0.7) ,
    //                            State(5, 6, des_states[5], STATE_TYPE::GATE, STATE_CHANGE_TYPE::DISTANCE, 1.0) ,
    //                            State(6, 7, des_states[6], STATE_TYPE::WAYPOINT, STATE_CHANGE_TYPE::DISTANCE, 0.7) ,
    //                            State(7, 8, des_states[7], STATE_TYPE::GATE, STATE_CHANGE_TYPE::DISTANCE, 1.0) ,
    //                            State(8, 1, des_states[8], STATE_TYPE::WAYPOINT, STATE_CHANGE_TYPE::DISTANCE, 0.7) ,
    //                          };

  


    StateMachine state_machine(nh, "/controller/reference", odom_sub_topic, "/state_machine/autonomy", states );

    ros::MultiThreadedSpinner multiSpinner(6);
    ros::spin(multiSpinner);

    return 0;
    
}