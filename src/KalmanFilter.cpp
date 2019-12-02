#include "racing_drone/KalmanFilter.hpp"
#include <iostream>

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

KalmanFilter::~KalmanFilter()
{
	
}

void KalmanFilter::predict(vector<double> u)
{
	if( std::isinf(X(0)) || std::isnan(X(0)) || std::isinf(u(0)))
		return;

	X = prod(F, X) + prod(B, u);
	PFt = prod(P, trans(F));
	P = prod(F, PFt) + Q;

}

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
