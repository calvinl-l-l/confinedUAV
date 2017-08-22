/**************************************/
/* EKLF Class */
/***************************************/

#ifndef __EKLF_INCLUDE__
#define __EKLF_INCLUDE__

// Includes
#include <iostream>

#include <cmath>
#include <vector>
#include <opencv2/core/core.hpp>

// EKLF Class
class EKLF {

public:
	// Constructor
	EKLF(const std::vector<float> &x_, const cv::Mat &P_, const cv::Mat &Q_, const cv::Mat &R_);

	// Destructor
	~EKLF();
	
private:	

	std::vector<float> x;	// State vector

	cv::Mat P;	// Prediction error covariance
	cv::Mat Q;	// Process noise covariance
	cv::Mat R;	// Measurement error covariance

	cv::Mat G;	// Kalman gain

	cv::Mat FJ;	// Jacobian of process model
	cv::Mat HJ;	// Jacobian of measurement model

	cv::Mat Pp;	// P Post prediction, pre-update
	
	cv::Mat Fx;	// State transition function
	cv::Mat Hx;	// Measurement function

	cv::Mat xMat;	// Measurement function
	cv::Mat zMat;	// Measurement function

	void update(const std::vector<float> &z, const cv::Mat &FJ, const cv::Mat &HJ);

	void calcPriorPredict(const cv::Mat &Fk_1, const cv::Mat &Pk_1, const cv::Mat &Qk_1);
	void calcKalmanGain(const cv::Mat &Pp, const cv::Mat &HJ, const cv::Mat &R);
	void calcEstimatedState(const cv::Mat &FJ, const cv::Mat &G, const cv::Mat &zMat, const cv::Mat &Hx);
	void calcPosteriorPredict(const cv::Mat &G, const cv::Mat &HJ, const cv::Mat &Pp);

};

#endif /*__EKLF_INCLUDE__*/ 
