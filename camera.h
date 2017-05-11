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
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;

class Camera {
public:
	Camera(int numDevice);
	~Camera();

	int cameraCalib(bool webcam);
	int cameraCorr();
	Mat subtractionBack(Mat image, Ptr<BackgroundSubtractor> pKNN);
	Mat circlesDetection(Mat image, Mat subImage);

	VideoCapture getCap();
	Mat getMap1();
	Mat getMap2();

private:
	VideoCapture cap;
	Mat intrinsicParam, distortionParam, map1, map2;
	vector<Vec3f> circles, previousCircles;
};