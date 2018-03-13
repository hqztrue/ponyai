#ifndef _PARAMETER_ESTIMATOR_H_
#define _PARAMETER_ESTIMATOR_H_
#include "util.h"

template<class T, class S>
class ParameterEstimator{
public:
	virtual void estimate(vector<T*> &data, vector<S> &parameters) = 0;
	virtual void leastSquaresEstimate(vector<T*> &data, vector<S> &parameters) = 0;
	virtual bool agree(vector<S> &parameters, T &data) = 0;
};

class PlaneParamEstimator : public ParameterEstimator<point3D, double>{
public:
	PlaneParamEstimator::PlaneParamEstimator(double delta) : m_deltaSquared(delta*delta) {}
	virtual void estimate(vector<point3D*> &data, vector<double> &parameters);
	virtual void leastSquaresEstimate(vector<point3D*> &data, vector<double> &parameters);
	virtual bool agree(vector<double> &parameters, point3D &data);
private:
	double m_deltaSquared;
};


#endif //_PARAMETER_ESTIMATOR_H_


