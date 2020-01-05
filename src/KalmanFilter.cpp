
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
KalmanFilter::KalmanFilter(matrix<double> f, matrix<double> b, matrix<double> h, matrix<double> q, matrix<double> r,  vector<double> x0):
	F(f), B(b), H(h), Q(q), R(r), X(x0)
{
	P.resize(f.size1(), f.size2());
	K.resize(f.size1(), h.size1());

	for(unsigned int i=0; i<P.size1(); ++i)
		P(i,i) = 100.0;
	
	PHt.resize(P.size1(), h.size1());
	PFt.resize(P.size1(), P.size1());
	HP.resize(h.size1(), P.size2());
	S.resize(r.size1(), r.size2());
	S_inv.resize(r.size1(), r.size2());

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


}

/**
 * @brief Destroy the Kalman Filter:: Kalman Filter object
 * 
 */
KalmanFilter::~KalmanFilter()
{	
}

/**
 * @brief Predict State in next time step and update the covariance matrix
 * 
 * @param u Control Input Vector
 */
void KalmanFilter::predict(vector<double> u)
{
	if( std::isinf(X(0)) || std::isnan(X(0)) || std::isinf(u(0)))
		return;

	X = prod(F, X) + prod(B, u);
	PFt = prod(P, trans(F));
	P = prod(F, PFt) + Q;

}

/**
 * @brief Fuse the measurements with prediction and update the covariance matrix
 * 
 * @param y 
 */
void KalmanFilter::update(vector<double> y)
{
	if( std::isinf(X(0)) || std::isnan(X(0)))
		return;
		

	PHt = prod(P, trans(H));
	S = prod(H, PHt) + R;
	
	for(unsigned int i=0; i < S.size1(); ++i)
		S_inv(i,i) = 1 / S(i,i);
		
	K = prod(PHt, S_inv);
	X = X + prod(K, (y - prod(H, X)) );
	HP = prod(H, P);
	P = P - prod(K, HP);
}


// ExtendedKalmanFilter::ExtendedKalmanFilter(matrix<double> f, matrix<double> b, matrix<double> h,		
// 	 					 matrix<double> q, matrix<double> r,  vector<double> x0,
// 						 EKFFuncType pFunc,
// 						 EKFFuncType jacoF,
// 						 EKFFuncType jacoB)
// 		: KalmanFilter(f,b,h,q,r,x0), predictFunc_(pFunc), jacobianF_(jacoF), jacobianB_(jacoB)
// {

// }

// ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

// vector<double> ExtendedKalmanFilter::predictEKF(vector<double> u)
// {
// 	X = predictFunc_(X,u);
// }
