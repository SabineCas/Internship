/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Kalman filter and estimators
*/

#include "kalman.h"

Kalman::Kalman()
{
	int stateSize = 4, measSize = 2, contrSize = 0;
	this->kf = cv::KalmanFilter(stateSize, measSize, contrSize, CV_32F);

	// State vector (x, y, vx, vy)
	this->state = cv::Mat(stateSize, 1, CV_32F);

	// Measurements vector (x, y)
	this->meas = cv::Mat(measSize, 1, CV_32F);
	this->kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	for (int i = 0; i < measSize; i++) {
		this->kf.measurementMatrix.at<float>(i, i) = float(1.0);
	}

	// Transition matrix that describes the relationship between model parameters at step k and at step k + 1
	cv::setIdentity(kf.transitionMatrix);

	// Process noise covariance
	this->kf.processNoiseCov.at<float>(0, 0) = float(0.01);
	this->kf.processNoiseCov.at<float>(1, 1) = float(0.01);
	this->kf.processNoiseCov.at<float>(2, 2) = float(5.0);
	this->kf.processNoiseCov.at<float>(3, 3) = float(5.0);

	/*for (int i = 0; i < measSize; i++) {
		this->kf.processNoiseCov.at<float>(i, i) = float(0.01);
	}*/

	// Measurement noise covariance
	cv::setIdentity(this->kf.measurementNoiseCov, cv::Scalar(0.1));
}

void Kalman::updateMeas(int x, int y)
{
	this->meas.at<float>(0, 0) = float(x);
	this->meas.at<float>(1, 0) = float(y);
}

void Kalman::initiateKalmanFilter()
{
	// Update the error covariance of the prediction
	this->kf.errorCovPre.at<float>(0, 0) = 1.0;
	this->kf.errorCovPre.at<float>(1, 1) = 1.0;
	this->kf.errorCovPre.at<float>(2, 2) = 1.0;
	this->kf.errorCovPre.at<float>(3, 3) = 1.0;

	// Update the state vector (x, y, vx, vy)
	this->state.at<float>(0, 0) = meas.at<float>(0, 0);
	this->state.at<float>(1, 0) = meas.at<float>(1, 0);
	this->state.at<float>(2, 0) = 0;
	this->state.at<float>(3, 0) = 0;

	// Update the state a posteriori
	this->kf.statePost = state;
}

void Kalman::predictKalmanFilter(clock_t dT) {
	// Correction Kalman filter
	this->kf.correct(this->meas);

	// Update the matrix F
	this->kf.transitionMatrix.at<float>(0, 2) = float(dT) / 1000;
	this->kf.transitionMatrix.at<float>(1, 3) = float(dT) / 1000;

	// Predict the next position
	this->state = this->kf.predict();
}

void Kalman::displayEstimatePosition(cv::Mat image)
{
	cv::circle(image, cv::Point(int(this->state.at<float>(0, 0)), int(this->state.at<float>(1, 0))), 5, cv::Scalar(255, 0, 0), -1);
}
