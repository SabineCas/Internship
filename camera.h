/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Camera calibration
*/

#pragma once

#define _CRT_SECURE_NO_DEPRECATE
#define DEBUG_CHESSBOARD

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "infraredLight.h"

// Value used for the threshold
const static int SENSITIVITY_VALUE = 30;
// Value used for the widening of the bounding box
const static int WIDE_BOUNDING_BOX_X = 20;

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
	std::vector<infraredLight> ledDetection(cv::Mat image, cv::Scalar lower, cv::Scalar upper);
	int ledFrequency(clock_t time, bool detected, bool previousDetected);
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
	clock_t getLEDTimeOFF();
	clock_t getLEDTimeON();

	// Setters
	void setBoundingBoxObs(int minX, int maxX, int minY, int maxY);
	void setDetectedCircle(bool det);
	

private:
	cv::VideoCapture cap;
	cv::Mat intrinsicParam, distortionParam, map1, map2;
	std::vector<cv::Vec3f> circles;
	cv::Rect2d boundingBoxObs;
	bool detectedCircle, detectedBlueLED;
	int nbDetectedLED;
	clock_t LEDTimeON, LEDTimeOFF;
};
