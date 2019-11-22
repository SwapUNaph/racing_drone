#pragma once

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;

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
