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

#include <ros/ros.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp> 
#include <boost/numeric/ublas/triangular.hpp> 
#include <boost/numeric/ublas/lu.hpp> 


using namespace boost::numeric::ublas;
namespace ublas = boost::numeric::ublas;

namespace RD
{
	class KalmanFilter
	{
	public:
		matrix<double> A;
		matrix<double> F;
		matrix<double> B;
		matrix<double> H;
		matrix<double> P;
		matrix<double> Q;
		matrix<double> R;
		matrix<double> K;
		ublas::vector<double> X;
		ublas::vector<double> X_dot;
		matrix<double> PHt;
		matrix<double> PFt;
		matrix<double> HP;
		matrix<double> S;
		matrix<double> S_inv;
		matrix<double> In;
		double dt;
		KalmanFilter(matrix<double> a, matrix<double> b, matrix<double> h,
					matrix<double> q, matrix<double> r,  double dt_);
		~KalmanFilter();
		void predict(ublas::vector<double> u);
		void update(ublas::vector<double> y);
		void filter(ublas::vector<double> u, ublas::vector<double> y);
	};
}


class ExtendedKalmanFilter : public RD::KalmanFilter
{
public:
	ExtendedKalmanFilter(matrix<double> a, matrix<double> b, matrix<double> h,
				 matrix<double> q, matrix<double> r,  double dt_);

	~ExtendedKalmanFilter();

	void calculateJacobianA(std::vector<double> w);
	void filter(std::vector<double> w, ublas::vector<double> u, ublas::vector<double> y);
	void predict(std::vector<double> w, ublas::vector<double> u);
	void update(std::vector<double> w, ublas::vector<double> y);
};

