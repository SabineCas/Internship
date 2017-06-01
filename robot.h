/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Robot
*/

#pragma once

#include "camera.h"
#include <stdio.h>

using namespace std;

class Robot {
public:
	Robot();
	void computeRealPosition();

	cv::Point2d getImagePosition();
	cv::Point3f getRealPosition();
	bool getBlueLEDDetected();

	void setImagePosition(int x, int y);
	void setRealPosition(float x, float y, float z);
	void setBlueLEDDetected(bool det);

private:
	cv::Point3f realPosition;
	cv::Point2d imagePosition;
	bool blueLEDDetected;
};
