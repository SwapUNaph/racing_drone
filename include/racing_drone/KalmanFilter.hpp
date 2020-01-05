/**
 * @file KalmanFilter.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief Kalman Filer and Extended Kalaman Filter class declaration
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

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <functional>

using namespace boost::numeric::ublas;

typedef std::function<matrix<double> (vector<double> , vector<double> )> EKFFuncType;

class KalmanFilter
{
public:
	matrix<double> F;
	matrix<double> B;
	matrix<double> H;
	matrix<double> P;
	matrix<double> Q;
	matrix<double> R;
	matrix<double> K;
	vector<double> X;
	matrix<double> PHt;
	matrix<double> PFt;
	matrix<double> HP;
	matrix<double> S;
	matrix<double> S_inv;
	KalmanFilter(matrix<double> f, matrix<double> b, matrix<double> h, matrix<double> q, matrix<double> r,  vector<double> x0);
	~KalmanFilter();
	void predict(vector<double> u);
	void update(vector<double> y);
};


// class ExtendedKalmanFilter : public KalmanFilter
// {
// public:
// 	EKFFuncType predictFunc_;
// 	EKFFuncType jacobianF_;
// 	EKFFuncType jacobianB_;

// 	ExtendedKalmanFilter(matrix<double> f, matrix<double> b, matrix<double> h,		
// 	 					 matrix<double> q, matrix<double> r,  vector<double> x0,
// 						 EKFFuncType pFunc,
// 						 EKFFuncType jacoF,
// 						 EKFFuncType jacoB);
// 	~ExtendedKalmanFilter();

// 	void predictEKF(vector<double> u);
// 	void updateEKF(vector<double> y);
// 	void calculateJacobians(void);
// };