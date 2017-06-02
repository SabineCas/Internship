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
	void updatePosition(cv::Point p1, cv::Point p2);
	cv::Mat displayImagePosition(cv::Mat image);

	cv::Point2d getImagePosition();
	cv::Point3f getRealPosition();

	void setImagePosition(int x, int y);
	void setRealPosition(float x, float y, float z);

private:
	cv::Point3f realPosition;
	cv::Point2d imagePosition;
};
