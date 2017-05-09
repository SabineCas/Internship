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
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class Camera {
public:
	Camera(int numDevice);
	~Camera();

	int cameraCalib(bool webcam);
	int cameraCorr();

private:
	VideoCapture cap;
	Mat intrinsicParam, distortionParam;
};