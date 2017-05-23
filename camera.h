/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Camera calibration
*/

#pragma once

#define _CRT_SECURE_NO_DEPRECATE
#define DEBUG_CHESSBOARD

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace std;

// Value used for the threshold
const static int SENSITIVITY_VALUE = 30;
// Value used for the widening of the bounding box
const static int WIDE_BOUNDING_BOX_X = 20;

// Proportional value of the LEDs
const static int  min_coef_LED = 10;
const static int max_coef_LED = 1;
// Proportional value of the robot
const static int  min_coef_robot = 10;
const static int max_coef_robot = 1;

class Camera {
public:
	// Constructor
	Camera(int numDevice);
	~Camera();

	int cameraCalib(bool webcam);
	int cameraCorr();
	cv::Mat updateBackground(cv::Mat image, cv::Ptr<cv::BackgroundSubtractor> pKNN);
	cv::Mat improveBackSubtr(cv::Mat image);
	cv::Mat colorDetection(cv::Mat image, cv::Scalar lower, cv::Scalar upper);
	std::vector<cv::Point> ledDetection(cv::Mat image, cv::Scalar lower, cv::Scalar upper);
	bool evaluateMarkersPosition(std::vector<cv::Point> blueVector);
	void circlesDetection(cv::Mat subImage);
	cv::Point coherenceCirclesMarkers(std::vector<cv::Point> blueVector);
	cv::Mat displayCircles(cv::Mat image);
	void wideringBoundingBox(int value);

	// Getters
	cv::VideoCapture getCap();
	cv::Rect2d getBoundingBoxObs();
	cv::Mat getMap1();
	cv::Mat getMap2();
	bool getDetectedCircle();
	bool getDetectedBlueLED();
	int getNbDetectedLED();

	// Setters
	void setBoundingBoxObs(int minX, int maxX, int minY, int maxY);
	void setDetectedCircle(bool det);
	

private:
	cv::VideoCapture cap;
	cv::Mat intrinsicParam, distortionParam, map1, map2;
	vector<cv::Vec3f> circles;
	cv::Rect2d boundingBoxObs;
	bool detectedCircle, detectedBlueLED;
	int nbDetectedLED;
};
