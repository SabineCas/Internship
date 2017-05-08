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

	int cameraCalib();
	int cameraCalibFromPict();
	int cameraCorr();

private:
	VideoCapture cap;
	Mat intrinsicParam, distortionParam;
};