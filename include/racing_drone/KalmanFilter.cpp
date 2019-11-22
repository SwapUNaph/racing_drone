#include "KalmanFilter.hpp"

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
}

KalmanFilter::~KalmanFilter()
{
	
}

void KalmanFilter::predict(vector<double> u)
{
	noalias(X) = prod(F, X) + prod(B, u);
	noalias(PFt) = prod(P, trans(F));
	noalias(P) = prod(F, PFt) + Q;
}

void KalmanFilter::update(vector<double> y)
{
	noalias(PHt) = prod(P, trans(H));
	noalias(S) = prod(H, PHt) + R;
	
	for(unsigned int i=0; i < S.size1(); ++i)
		S_inv(i,i) = 1 / S(i,i);
		
	noalias(K) = prod(PHt, S_inv);
	noalias(X) = X + prod(K, (y - prod(H, X)) );
	noalias(HP) = prod(H, P);
	noalias(P) = P - prod(K, HP);
}
