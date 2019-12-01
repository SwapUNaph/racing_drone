#include "racing_drone/MPC.hpp"

//Position constraints
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

//Velocity constraints
double dynamic_constraint_velocity(const std::vector<double> &x, std::vector<double> &grad, void *vel_const_data)
{ 
	vel_const_struct* vel_struct = reinterpret_cast<vel_const_struct*>(vel_const_data);
	unsigned int i = vel_struct->i;
	unsigned int k = vel_struct->k;
	double dt = vel_struct->dt;
	double psi = vel_struct->psi;
	unsigned int Ns = vel_struct->Ns;
	double drag_term = 0.005;
		
	//std::cout << "In vel const function" << std::endl;
	
	switch(k)
	{
		case 3:
		if(!grad.empty())
		{
			std::fill(grad.begin(), grad.end(), 0.0);
			grad[Ns*i+k] = -1.0;
			grad[Ns*(i-1)+k] = 1.0 - drag_term * std::abs(x[Ns*(i-1)+k]) * dt;
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
		
		case 4:
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
		
		case 5:
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
		return -1;
		break;
	}
	
}


//Endpoint constraints
double end_point_constraints(const std::vector<double> &x, std::vector<double> &grad, void* ep_const_data)
{
	end_pnt_const_struct* ep_data = reinterpret_cast<end_pnt_const_struct* >(ep_const_data);
	unsigned int ind = ep_data->i;
	double val = ep_data->x;
		
	// std::cout << "In end point const function, before grad i,k: " << ep_data->i << ", " << ep_data->x << std::endl;
	
	if(!grad.empty())
	{
		std::fill(grad.begin(), grad.end(), 0.0);
		grad[ind] = 1.0;
	}
	// std::cout << "In end point const function, after grad i,k: " << ind << ", " << val << std::endl;
	return x[ind] - val;
}



// Cost function
double cost_function(const std::vector<double> &x, std::vector<double> &grad, void* cost_func_data)
{
	std::vector<double> *q_ref = reinterpret_cast<std::vector<double>*>(cost_func_data);
	unsigned int Ns = (q_ref->size())/2;
	std::vector<double> Q(Ns);
	std::vector<double> ref(Ns);
	std::copy(q_ref->begin(), q_ref->end() - Ns, Q.begin());
	std::copy(q_ref->begin() + Ns, q_ref->end(), ref.begin());
	
	//std::cout << "In cost function" << std::endl;
	
	double cost = 0.0;
	
	unsigned int N = x.size() / Ns;
	
	if(!grad.empty())
		for(unsigned int i=0; i < N; i++)
			for(unsigned int k=0; k < Ns; k++)
				grad[Ns*i + k] = 2.0 * (x[Ns*i + k] - ref[k]) * Q[k];
	
	for(unsigned int i=0; i < N; i++)
		for(unsigned int k=0; k < Ns; k++)
			cost += (x[Ns*i + k] - ref[k]) * (x[Ns*i + k] - ref[k]) * Q[k];
	
	// std::cout << "Cost: " << cost << std::endl;
	return cost;
}

//state: x0, y0, z0, vx0, vy0, vz0, th0, phi0, T0, ..., xN, yN, zN, vxN, vyN, vzN, thN, phiN, TN,   : 9*N states

MPC::MPC(unsigned int n, std::vector<double> p, double dt_): N(n), P(p), dt(dt_)
{
    Ns = 9;
    sol_x.resize(N*Ns);
}

MPC::~MPC(){}

int MPC::optimize(std::vector<double> x0, std::vector<double> xN, double psi)
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
	
	std::vector<double> Q_ref(2*Ns); //Penalties for states and inputs (Ns) and reference (Ns)
	std::copy(P.begin(), P.end(), Q_ref.begin()); // Fill all penalties with P
	std::copy(xN.begin(), xN.end(), Q_ref.begin() + Ns); // Copy reference in the cost func input data
	
	//set Cost function
	OPT.set_min_objective(cost_function, &Q_ref);
	
 
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
			lb[Ns*i + k] = -30.0 * PI / 180.0;
			ub[Ns*i + k] =  30.0 * PI / 180.0;
		}
		
		//Thrust bounds
		lb[Ns*i + Ns-1] = 0.0;
		ub[Ns*i + Ns-1] = 15.0;
		
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
 
	try
	{
		// std::cout << "Optimization started..." << std::endl;
		OPT.optimize(sol_x, minf);
		return EXIT_SUCCESS;
	}
	catch(std::exception &e) 
	{
		std::cout << "nlopt failed: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}