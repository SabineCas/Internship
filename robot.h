/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Robot
*/

#pragma once

#include "camera.h"
#include "kalman.h"
#include <stdio.h>

using namespace std;

class Robot {
public:
	//! Constructor by default.
	Robot(double H, double alphaU, double alphaV, double u0, double v0);

	//! Update the position of the robot on the picture (cv::Point2d imagePosition attribut) accordings to the two detected LEDs
	//! passed as parameters. 
	/*!
	\param p1 First detected LED coordinate
	\param p2 Second detected LED coordinate
	\return void
	*/
	void updatePosition(cv::Point p1, cv::Point p2);

	//! Compute the angle between the position of the robot and the cv::Point parameter that the robot wants to reach. 
	/*!
	\param p Coordinate that the robot want to reach
	\return The difference angle between the orientation of the robot and the point p
	*/
	double calculateRotation(cv::Point p);

	//! Compute the distance between the position of the robot and the cv::Point parameter that the robot wants to reach.
	//! if the returned value is < 0, then the robot has to turn right, otherwise the robot has to turn left.
	/*!
	\param p Coordinate that the robot want to reach
	\return The distance angle between the orientation of the robot and the point p
	*/
	double calculateDistance(cv::Point p);

	//! Display a filled circle on the currect frame at the coordinate saved into the cv::Point2d imagePosition attribut.
	/*!
	\param image The current frame
	\return The current frame with the circle drawn
	*/
	void displayImagePosition(cv::Mat image);

	//! Display an arrow on the currect frame at the coordinate saved into the cv::Point2d imagePosition attribut that represents the
	//! orientation of the robot.
	/*!
	\param image The current frame
	\return The current frame with the arrow drawn
	*/
	void displayImageOrientation(cv::Mat image, cv::Point top);

	//! Return the image coordinate of the robot saved into the cv::Point2d imagePosition attribut.
	/*!
	\param void
	\return The cv::Point2d imagePosition attribut
	*/
	cv::Point2d getImagePosition();

	//! Return the coordinate of the robot into the world coordinate system saved into the cv::Point3f realPosition attribut.
	/*!
	\param void
	\return The cv::Point3f realPosition attribut
	*/
	cv::Point3f getRealPosition();

private:
	cv::Point3f realPosition;
	cv::Point2d imagePosition;
	double H, alphaU, alphaV, u0, v0, angleOrientation;
};
