
/**
 * @file KalmanFilter.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief Kalman Filter class definition
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

#include "racing_drone/KalmanFilter.hpp"
#include <iostream>

bool InvertMatrix (const matrix<double>& input, matrix<double>& inverse) { 
    // using namespace boost::numeric::ublas; 
    typedef permutation_matrix<std::size_t> pmatrix; 
    // create a working copy of the input 
    matrix<double> A(input); 
    // create a permutation matrix for the LU-factorization 
    pmatrix pm(A.size1()); 
    // perform LU-factorization 
    int res = lu_factorize(A,pm); 
    if( res != 0 )
        return false; 
    // create identity matrix of "inverse" 
    inverse.assign(identity_matrix<double>(A.size1())); 
    // backsubstitute to get the inverse 
    lu_substitute(A, pm, inverse); 
    return true; 
}


/**
 * @brief Construct a new Kalman Filter:: Kalman Filter object
 * 
 * @param f n-by-n System Dynamics Matrix (n = no. of states)  
 * @param b n-by-m Control Dynamics Matrix (m = no. of control inputs)
 * @param h p-by-n Measurement Matrix (p = no. of measurements)
 * @param q n-by-n State Noise Covariance Matrix
 * @param r p-by-p Measurement Noise Covariance Matrix
 * @param x0 n dimensional Initial State Estimate Vector
 */
RD::KalmanFilter::KalmanFilter(matrix<double> a, matrix<double> b, matrix<double> h,
				 matrix<double> q, matrix<double> r,  double dt_):
	A(a), B(b), H(h), Q(q), R(r),dt(dt_)
{
	P.resize(A.size1(), A.size2());
	In.resize(A.size1(), A.size2());
	K.resize(A.size1(), H.size1());

	for(unsigned int i=0; i<P.size1(); ++i)
		P(i,i) = 100.0;
	
	X.resize(A.size1());
	X_dot.resize(A.size1());
	F.resize(A.size1(), A.size2());
	PHt.resize(P.size1(), H.size1());
	PFt.resize(P.size1(), P.size1());
	HP.resize(H.size1(), P.size2());
	S.resize(R.size1(), R.size2());
	S_inv.resize(R.size1(), R.size2());

	for(int i=0; i<PHt.size1(); i++)
		for(int j=0; j<PHt.size2(); j++)
			PHt(i,j) = 0.0;

	for(int i=0; i<PFt.size1(); i++)
		for(int j=0; j<PFt.size2(); j++)
			PFt(i,j) = 0.0;

	for(int i=0; i<HP.size1(); i++)
		for(int j=0; j<HP.size2(); j++)
			HP(i,j) = 0.0;

	for(int i=0; i<S.size1(); i++)
		for(int j=0; j<S.size2(); j++)
			S(i,j) = 0.0;

	for(int i=0; i<S_inv.size1(); i++)
		for(int j=0; j<S_inv.size2(); j++)
			S_inv(i,j) = 0.0;

	for(int i=0; i<In.size1(); i++)
		for(int j=0; j<In.size2(); j++)
		{
			if(i == j)
				In(i,j) = 1.0;
			else
				In(i,j) = 0.0;
			
		}

	for(int i=0; i<X.size(); i++)
		X(i) = 0.0;

	X_dot = X;
}

/**
 * @brief Destroy the Kalman Filter:: Kalman Filter object
 * 
 */
RD::KalmanFilter::~KalmanFilter()
{	
}

/**
 * @brief Predict State in next time step and update the covariance matrix
 * 
 * @param u Control Input Vector
 */
void RD::KalmanFilter::predict(vector<double> u)
{
	ublas::vector<double> X_dot_curr( prod(A, X) + prod(B, u) );
	noalias(X) = X + (X_dot + X_dot_curr) / 2.0 * dt;
	noalias(X_dot) = X_dot_curr;
	noalias(F) = In + A * dt;
	noalias(PFt) = prod(P, trans(F));
	noalias(P) = prod(F, PFt) + Q;
}

/**
 * @brief Fuse the measurements with prediction and update the covariance matrix
 * 
 * @param y 
 */
void RD::KalmanFilter::update(vector<double> y)
{
	noalias(PHt) = prod(P, trans(H));
	noalias(S) = prod(H, PHt) + R;
	
	for(unsigned int i=0; i < S.size1(); ++i)
		S_inv(i,i) = 1.0 / S(i,i);
	
	noalias(K) = prod(PHt, S_inv);
	noalias(X) = X + prod(K, (y - prod(H, X)) );
	noalias(HP) = prod(H, P);
	noalias(P) = P - prod(K, HP);
}

void RD::KalmanFilter::filter(vector<double> u, vector<double> y)
{
	predict(u);
	update(y);
}


ExtendedKalmanFilter::ExtendedKalmanFilter(matrix<double> a, matrix<double> b, matrix<double> h,		
	 					 matrix<double> q, matrix<double> r,  double dt_)
		: RD::KalmanFilter(a,b,h,q,r,dt_)
{

}

ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

void ExtendedKalmanFilter::calculateJacobianA(std::vector<double> w)
{
	A(0,0) =   0.0; A(0,1) =  w[2];	A(0,2) = -w[1];	   // A(0,3) = -1.0;	A(0,4) =  0.0;	A(0,5) =  0.0;	
	A(1,0) = -w[2]; A(1,1) =   0.0; A(1,2) =  w[0];	   // A(1,3) =  0.0;	A(1,4) = -1.0;	A(1,5) =  0.0;
	A(2,0) =  w[1]; A(2,1) = -w[0]; A(2,2) =   0.0;	   // A(2,3) =  0.0;	A(2,4) =  0.0;	A(2,5) = -1.0;

	// A(3,0) =   0.0; A(3,1) =   0.0;	A(3,2) =   0.0; 	A(3,3) =  0.0;	A(3,4) =  0.0;	A(3,5) =  0.0;	
	// A(4,0) =   0.0; A(4,1) =   0.0; A(4,2) =   0.0; 	A(4,3) =  0.0;	A(4,4) =  0.0;	A(4,5) =  0.0;
	// A(5,0) =   0.0; A(5,1) =   0.0; A(5,2) =   0.0; 	A(5,3) =  0.0;	A(5,4) =  0.0;	A(5,5) =  0.0;

}

void ExtendedKalmanFilter::filter(std::vector<double> w, vector<double> u, vector<double> y)
{
	calculateJacobianA(w);
	RD::KalmanFilter::filter(u,y);
}

void ExtendedKalmanFilter::predict(std::vector<double> w, ublas::vector<double> u)
{
	calculateJacobianA(w);
	RD::KalmanFilter::predict(u);
}

void ExtendedKalmanFilter::update(std::vector<double> w, ublas::vector<double> y)
{
	calculateJacobianA(w);
	RD::KalmanFilter::update(y);
}