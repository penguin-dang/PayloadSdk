#ifndef SIMPLE_KALMAN_FILTER_H
#define SIMPLE_KALMAN_FILTER_H

class SimpleKalmanFilter
{

public:
	SimpleKalmanFilter(float mea_e, float est_e, float q);
	float updateEstimate(float mea);
	void setMeasurementError(float mea_e);
	void setEstimateError(float est_e);
	void setProcessNoise(float q);
	void setLastEstimate(float last_estimate);
	float getKalmanGain();
	float getEstimateError();
	void reset();

private:
	float _err_measure;
	float _err_estimate;
	float _q;
	float _current_estimate = 0;
	float _last_estimate = 0;
	float _kalman_gain = 0;

};

#endif /* SIMPLEKALMANFILTER_H */
