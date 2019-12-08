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


class ExtendedKalmanFilter : public KalmanFilter
{
public:
	EKFFuncType predictFunc_;
	EKFFuncType jacobianF_;
	EKFFuncType jacobianB_;

	ExtendedKalmanFilter(matrix<double> f, matrix<double> b, matrix<double> h,		
	 					 matrix<double> q, matrix<double> r,  vector<double> x0,
						 EKFFuncType pFunc,
						 EKFFuncType jacoF,
						 EKFFuncType jacoB);
	~ExtendedKalmanFilter();

	void predictEKF(vector<double> u);
	void updateEKF(vector<double> y);
	void calculateJacobians(void);
};