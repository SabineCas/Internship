/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Kalman filter and estimators
*/

#pragma once
#include <opencv2\opencv.hpp>

//! The Kalman class manages the estimation of the robot position using the information from the detection algorithm and the command send to the robot.
class Kalman {
public:
	//! Constructor by default
	Kalman();

	//! Update the measurements vector of the Kalman filter
	/*!
	\param x Position in the x axis of the picture
	\param y Position in the y axis of the picture
	\return void
	*/
	void updateMeas(int x, int y);

	//! Function that have to be called when this is the first time that the robot is detected.
	// This function initiate the Kalman filter.
	/*!
	\param void
	\return void
	*/
	void initiateKalmanFilter();

	//! Predict the position according to the control vecteur and previous data
	/*!
	\param dT Time past since the last frame
	\return void
	*/
	void predictKalmanFilter(clock_t dT);

	//! Display the estimator on the picture passed as a parameter
	/*!
	\param image The current frame where the estimator will be displayed
	\return void
	*/
	void displayEstimatePosition(cv::Mat image);

private:
	cv::KalmanFilter kf;
	cv::Mat state, meas;
};