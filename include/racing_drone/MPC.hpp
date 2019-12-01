#pragma once

#include <iostream>
#include <iomanip>
#include <cmath>
#include <nlopt.hpp>
#include <nlopt.h>

#define PI 3.14159265359

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
        std::vector<double> sol_x; //Optimization State Vector
        std::vector<double> P;  //Cost penalty weights
        double dt; //Prediction Time step

        MPC(unsigned int n, std::vector<double> p, double dt_);
        ~MPC();

        int optimize(std::vector<double> x0, std::vector<double> xN, double psi);
};