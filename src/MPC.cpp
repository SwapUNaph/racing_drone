/**
 * @file MPC.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief UAV Non-linear Model Predicitve Control Problem formulation and solution
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

#include "racing_drone/MPC.hpp"

/**
 * @brief Position constraint Definition
 * 
 * @param x Optimization Vector [x0, y0, z0, vx0, vy0, vz0, pitch0, roll0, T0, ..., xN, yN, zN, vxN, vyN, vzN, pitchN, rollN, TN]
 * @param grad Gradient Vector
 * @param pos_const_data Position constraint data
 * @return double Constraint residue
 */
double dynamic_constraint_position(const std::vector<double> &x, std::vector<double> &grad, void *pos_const_data)
{ 
	pos_const_struct *pos_struct = reinterpret_cast<pos_const_struct*>(pos_const_data);
	unsigned int ind = pos_struct->i;
	unsigned int k = pos_struct->k;
	unsigned int Ns = pos_struct->Ns;
	double dt = pos_struct->dt;
		
	//std::cout << "In pos const function" << std::endl;
	
	if(!grad.empty())
	{
		fill(grad.begin(), grad.end(), 0.0);
		grad[Ns*ind+k] = -1.0;
		grad[Ns*(ind-1)+k] = 1.0;
		grad[Ns*(ind-1)+k+3] = dt;
	}
	return -x[Ns*ind+k] + x[Ns*(ind-1)+k] + x[Ns*(ind-1)+k+3] * dt;
}

/**
 * @brief Velocity constraint Definition
 * 
 * @param x Optimization Vector [x0, y0, z0, vx0, vy0, vz0, pitch0, roll0, T0, ..., xN, yN, zN, vxN, vyN, vzN, pitchN, rollN, TN]
 * @param grad Gradient Vector
 * @param vel_const_data Velocity constraint data
 * @return double Constraint residue
 */
double dynamic_constraint_velocity(const std::vector<double> &x, std::vector<double> &grad, void *vel_const_data)
{ 
	vel_const_struct* vel_struct = reinterpret_cast<vel_const_struct*>(vel_const_data);
	unsigned int i = vel_struct->i;
	unsigned int k = vel_struct->k;
	double dt = vel_struct->dt;
	double psi = vel_struct->psi;
	unsigned int Ns = vel_struct->Ns;
	double drag_term = 0.0;
		
	//std::cout << "In vel const function" << std::endl;
	
	switch(k)
	{
		case 3:  // vx
		if(!grad.empty())
		{
			std::fill(grad.begin(), grad.end(), 0.0);
			grad[Ns*i+k] = -1.0;
			grad[Ns*(i-1)+k] = 1.0 - 2.0 * drag_term * std::abs(x[Ns*(i-1)+k]) * dt;
			grad[Ns*(i-1) + Ns-3] = x[Ns*(i-1) + Ns-1] * ( std::cos(x[Ns*(i-1) + Ns-2]) * std::cos(x[Ns*(i-1) + Ns-3]) * std::cos(psi) ) * dt;
			grad[Ns*(i-1) + Ns-2] = x[Ns*(i-1) + Ns-1] * ( std::cos(x[Ns*(i-1) + Ns-2]) * std::sin(psi) - std::sin(x[Ns*(i-1) + Ns-2]) * std::sin(x[Ns*(i-1) + Ns-3]) * std::cos(psi) ) * dt;
			grad[Ns*(i-1) + Ns-1] = ( std::sin(x[Ns*(i-1) + Ns-2]) * std::sin(psi) + std::cos(x[Ns*(i-1) + Ns-2]) * std::sin(x[Ns*(i-1) + Ns-3]) * std::cos(psi) ) * dt;
		}
		return  ( -x[Ns*i + k] + x[Ns*(i-1)+k] + 
					(
						x[Ns*(i-1) + Ns-1] * ( std::sin(x[Ns*(i-1) + Ns-2]) * std::sin(psi) + std::cos(x[Ns*(i-1) + Ns-2]) * std::sin(x[Ns*(i-1) + Ns-3]) * std::cos(psi) )
						 - drag_term * std::abs(x[Ns*(i-1)+k]) * x[Ns*(i-1)+k]
					) * dt
				);
		break;
		
		case 4: // vy
		if(!grad.empty())
		{
			std::fill(grad.begin(), grad.end(), 0.0);
			grad[Ns*i+k] = -1.0;
			grad[Ns*(i-1)+k] = 1.0 - drag_term * std::abs(x[Ns*(i-1)+k]) * dt;
			grad[Ns*(i-1) + Ns-3] = x[Ns*(i-1) + Ns-1] * ( std::cos(x[Ns*(i-1) + Ns-2]) * std::cos(x[Ns*(i-1) + Ns-3]) * std::sin(psi) ) * dt;
			grad[Ns*(i-1) + Ns-2] = x[Ns*(i-1) + Ns-1] * ( -std::cos(x[Ns*(i-1) + Ns-2]) * std::cos(psi) - std::sin(x[Ns*(i-1) + Ns-2]) * std::sin(x[Ns*(i-1) + Ns-3]) * std::sin(psi) ) * dt;
			grad[Ns*(i-1) + Ns-1] = ( -std::sin(x[Ns*(i-1) + Ns-2]) * std::cos(psi) + std::cos(x[Ns*(i-1) + Ns-2]) * std::sin(x[Ns*(i-1) + Ns-3]) * std::sin(psi) ) * dt;
		}
		return  ( -x[Ns*i + k] + x[Ns*(i-1)+k] + 
					(
						x[Ns*(i-1) + Ns-1] * ( -std::sin(x[Ns*(i-1) + Ns-2]) * std::cos(psi) + std::cos(x[Ns*(i-1) + Ns-2]) * std::sin(x[Ns*(i-1) + Ns-3]) * std::sin(psi) )
						 - drag_term * std::abs(x[Ns*(i-1)+k]) * x[Ns*(i-1)+k]
					) * dt
				);
		break;
		
		case 5: // vz
		if(!grad.empty())
		{
			std::fill(grad.begin(), grad.end(), 0.0);
			grad[Ns*i+k] = -1.0;
			grad[Ns*(i-1)+k] = 1.0 - drag_term * std::abs(x[Ns*(i-1)+k]) * dt;
			grad[Ns*(i-1) + Ns-3] = x[Ns*(i-1) + Ns-1] * ( -std::cos(x[Ns*(i-1) + Ns-2]) * std::sin(x[Ns*(i-1) + Ns-3]) ) * dt;
			grad[Ns*(i-1) + Ns-2] = x[Ns*(i-1) + Ns-1] * ( -std::sin(x[Ns*(i-1) + Ns-2]) * std::cos(x[Ns*(i-1) + Ns-3]) ) * dt;
			grad[Ns*(i-1) + Ns-1] = ( std::cos(x[Ns*(i-1) + Ns-2]) * std::cos(x[Ns*(i-1) + Ns-3]) ) * dt;
		}
		return  ( -x[Ns*i + k] + x[Ns*(i-1)+k] + 
					(
						x[Ns*(i-1) + Ns-1] * ( std::cos(x[Ns*(i-1) + Ns-2]) * std::cos(x[Ns*(i-1) + Ns-3]) )
						 - drag_term * std::abs(x[Ns*(i-1)+k]) * x[Ns*(i-1)+k] - 9.80
					) * dt
				);
		break;
		
		default:
		return 0;
		break;
	}
	
}


/**
 * @brief End point constraint Definition
 * 
 * @param x Optimization Vector [x0, y0, z0, vx0, vy0, vz0, pitch0, roll0, T0, ..., xN, yN, zN, vxN, vyN, vzN, pitchN, rollN, TN]
 * @param grad Gradient Vector
 * @param ep_const_data End-point constraint data
 * @return double Constraint residue
 */
double end_point_constraints(const std::vector<double> &x, std::vector<double> &grad, void* ep_const_data)
{
	end_pnt_const_struct* ep_data = reinterpret_cast<end_pnt_const_struct* >(ep_const_data);
	unsigned int ind = ep_data->i;
	double val = ep_data->x;
	
	if(!grad.empty())
	{
		std::fill(grad.begin(), grad.end(), 0.0);
		grad[ind] = 1.0;
	}

	return x[ind] - val;
}



/**
 * @brief Cost Function Definition [xQx.T + uRu.T]
 * 
 * @param x Optimization Vector [x0, y0, z0, vx0, vy0, vz0, pitch0, roll0, T0, ..., xN, yN, zN, vxN, vyN, vzN, pitchN, rollN, TN]
 * @param grad Gradient Vector
 * @param pos_const_data Position constraint data
 * @return double Constraint residue
 */
double cost_function(const std::vector<double> &x, std::vector<double> &grad, void* cost_func_data)
{
	std::vector<double> *q_ref = reinterpret_cast<std::vector<double>*>(cost_func_data);
	unsigned int Ns = (q_ref->size())/2;
	std::vector<double> Q(Ns);
	std::vector<double> ref(Ns);
	std::copy(q_ref->begin(), q_ref->end() - Ns, Q.begin());
	std::copy(q_ref->begin() + Ns, q_ref->end(), ref.begin());
	double cost = 0.0;
	unsigned int N = x.size() / Ns;

	// std::cout << "In MPC cost function N: " << N << ", Ns: " << Ns << std::endl;
	// ROS_INFO("\nReference x: %f, y: %f, z: %f\n vx: %f, vy: %f, vz: %f\n R: %f, P: %f, T: %f\n", ref[0], ref[1], ref[2],
	// 			ref[3], ref[4], ref[5], ref[6], ref[7], ref[8]);

	if(!grad.empty())
		for(unsigned int i=0; i < N; i++)
			for(unsigned int k=0; k < Ns; k++)
				grad[Ns*i + k] = 2.0 * (x[Ns*i + k] - ref[k]) * Q[k];
	
	for(unsigned int i=0; i < N; i++)
		for(unsigned int k=0; k < Ns; k++)
			cost += (x[Ns*i + k] - ref[k]) * (x[Ns*i + k] - ref[k]) * Q[k];
	
	cost += 500.0;
	// std::cout << "\nCost: " << cost << std::endl;
	return cost;
}

//state: x0, y0, z0, vx0, vy0, vz0, th0, phi0, T0, ..., xN, yN, zN, vxN, vyN, vzN, thN, phiN, TN,   : 9*N states

/**
 * @brief Construct a new MPC::MPC object
 * 
 * @param n Prediction horizon
 * @param p State and Control Input penalties (1-by-9 vector)
 * @param dt_ Time step (in seconds)
 */
MPC::MPC(unsigned int n, std::vector<double> p, double dt_): N(n), P(p), dt(dt_)
{
    Ns = 9;
    sol_x.resize(N*Ns);
}

/**
 * @brief Destroy the MPC::MPC object
 * 
 */
MPC::~MPC(){}


/**
 * @brief Solve the optimization problem using SLSQP method
 * 
 * @param x0 Initial State
 * @param xN Reference State
 * @param psi Current yaw angle
 * @return int 0 for success and 1 for failure of optimization
 */
int MPC::optimize(std::vector<double>& x0, std::vector<double>& xN, double& psi)
{
    // auto start = std::chrono::system_clock::now();
	// unsigned int Ns = 9; //States
	// unsigned int N = 10; //Prediction steps
	// double dt = 0.1; // Time step in s
	// std::vector<double> x0(Ns-3); // Start state vector
	// std::vector<double> xN(Ns-3); // Goal state vector
	// xN[0] = 1.0;
	// xN[1] = 1.0;
	// xN[2] = 0.0;
	
	// double psi = 0.0;
	
	nlopt::opt OPT(nlopt::LD_SLSQP, Ns*N);
	
	std::vector<double> P_ref(2*Ns); //Penalties for states and inputs (Ns) and reference (Ns)
	std::copy(P.begin(), P.end(), P_ref.begin()); // Fill all penalties with P
	std::copy(xN.begin(), xN.end(), P_ref.begin() + Ns); // Copy reference in the cost func input data
	
	//set Cost function
	OPT.set_min_objective(cost_function, &P_ref);
	
 
	// Upper and Lower bounds
	std::vector<double> lb(Ns*N);
	std::vector<double> ub(Ns*N);
	std::fill(lb.begin(), lb.end(), -100.0);
	std::fill(ub.begin(), ub.end(), 100.0);
	
	for(unsigned int i = 0; i < N; i++)
	{
		// Input angle bounds
		for(unsigned int k = Ns-3; k < Ns-1; k++)
		{
			lb[Ns*i + k] = -80.0 * PI / 180.0;
			ub[Ns*i + k] =  80.0 * PI / 180.0;
		}
		
		//Thrust bounds
		lb[Ns*i + 8] = 0.0;
		ub[Ns*i + 8] = 15.0;
		
	}
	OPT.set_lower_bounds(lb);
	OPT.set_upper_bounds(ub);
	
	//Constraint structs
	pos_const_struct pos_str[N*3];
	vel_const_struct vel_str[N*3];
	end_pnt_const_struct ep_str[2*(Ns - 3)];


	//Position Dynamics constriants
	for(unsigned int i=1; i < N; i++)
	{
		for(unsigned int k = 0; k < 3; k++)
		{
			pos_str[i*3 + k] = {.i = i, .k = k, .N = N, .Ns = Ns, .dt = dt};
			OPT.add_equality_constraint(dynamic_constraint_position, &pos_str[i*3 + k], 1e-2);
		}
	}
	
	//Velocity Dynamics constriants
	for(unsigned int i=1; i < N; i++)
	{
		for(unsigned int k = 3; k < 6; k++)
		{
			vel_str[i*3 + k - 3] = {.i = i, .k = k, .N = N, .Ns = Ns, .dt = dt, .psi = psi};
			OPT.add_equality_constraint(dynamic_constraint_velocity, &vel_str[i*3 + k - 3], 1e-2);
		}
	}
	
	//Start and end point constraints
	for(unsigned int i = 0; i < Ns - 3; i++)
	{
		// std::cout << "end point definition: i: " << i << std::endl;
		ep_str[i] = {.i = i, .x = x0[i]};
		OPT.add_equality_constraint(end_point_constraints, &ep_str[i], 1e-2);
		
		// std::cout << "end point definition: (Ns*(N-1) + i): " << (Ns*(N-1) + i) << std::endl;
		// ep_str[2*i] = {.i = (Ns*(N-1) + i), .x = xN[i]};
		// opt.add_equality_constraint(end_point_constraints, &ep_str[2*i], 1e-4);
	}
	
	
	//State vector
	// std::vector<double> x(Ns*N);
	OPT.set_xtol_rel(1e-2);
	double minf;
	std::vector<double> tmp = sol_x;

	try
	{
		// std::cout << "Optimization started..." << std::endl;
		OPT.optimize(sol_x, minf);
		return EXIT_SUCCESS;
	}
	catch(std::exception &e) 
	{
		std::cout << "nlopt failed: " << e.what() << std::endl;
		sol_x = tmp;
		return EXIT_FAILURE;
	}
}