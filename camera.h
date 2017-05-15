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
//#include <opencv2/tracking.hpp>

using namespace cv;
using namespace std;

// Value used for the threshold
const static int SENSITIVITY_VALUE = 30;
// Value used for the widening of the bounding box
const static int WIDE_BOUNDING_BOX_X = 20;

class Camera {
public:
	Camera(int numDevice);
	~Camera();

	int cameraCalib(bool webcam);
	int cameraCorr();
	Mat subtractionBack(int solution, Mat image, Ptr<BackgroundSubtractor> pKNN, Mat previousSubImage);
	Mat colorDetection(int limit, Mat image);
	void circlesDetection(Mat image, Mat subImage);
	void circlesDetection(Mat image, Mat subImage, Mat thresImage);
	Mat displayCircles(Mat image);
	//Mat objectTracking(Mat image, Ptr<Tracker> tracker, Rect2d bbox);

	VideoCapture getCap();
	Rect2d getBoundingBoxObs();
	Mat getMap1();
	Mat getMap2();

	void setBoundingBoxObs(int minX, int maxX, int minY, int maxY);
	void wideringBoundingBox(int value);

private:
	VideoCapture cap;
	Mat intrinsicParam, distortionParam, map1, map2;
	vector<Vec3f> circles;
	Rect2d boundingBoxObs;
	bool detectedCircle;
};
