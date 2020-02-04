/**
 * @file MPC.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief UAV Model Predictive Control class declaration
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

#include <iostream>
#include <iomanip>
#include <cmath>
#include <nlopt.hpp>
#include <nlopt.h>
#include <ros/ros.h>

#define PI 3.14159265359
#define G 9.80

typedef struct{
	unsigned int i, k, N, Ns;
	double dt;
}pos_const_struct;

typedef struct{
	unsigned int i, k, N, Ns;
	double dt, psi;
}vel_const_struct;

typedef struct{
	unsigned int i;
	double x;
}end_pnt_const_struct;

double dynamic_constraint_position(const std::vector<double> &x, std::vector<double> &grad, void *pos_const_data);
double dynamic_constraint_velocity(const std::vector<double> &x, std::vector<double> &grad, void *vel_const_data);
double end_point_constraints(const std::vector<double> &x, std::vector<double> &grad, void* ep_const_data);
double cost_function(const std::vector<double> &x, std::vector<double> &grad, void* cost_func_data);

class MPC
{
    public:
        unsigned int N;  //Prediction horizon
        unsigned int Ns; //Number of states: 9
        std::vector<double> sol_x; //Optimization State Vector -> x,y,z,vx,vy,vz,pitch,roll,thrust
        std::vector<double> P;  //Cost penalty weights
        double dt; //Prediction Time step
        double max_angle;
        double max_thrust_accel;

        MPC(unsigned int n, std::vector<double> p, double dt_, double maxAng, double maxThrust);
        ~MPC();

        int optimize(std::vector<double>& x0, std::vector<double>& xN, double& psi);
};