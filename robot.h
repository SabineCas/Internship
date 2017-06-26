/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Robot
*/

#pragma once

#include "camera.h"
#include "kalman.h"

#include <QSerialPort>
#include <stdio.h>
#include <conio.h>
#include <windows.h>
#include <tchar.h>
//#include <sdkddkver.h>

class Robot {
public:
	//! Constructor that take as parameters the height of the camera and the parameters of the camera.
	/*!
	\param H Height of the camera
	\param alphaU The real size of the pixel on the X axis
	\param alphaV The real size of the pixel on the Y axis
	\param u0 The center of the picture in the image plan on the X axis
	\param v0 The center of the picture in the image plan on the Y axis
	\return void
	*/
	Robot(double H, double alphaU, double alphaV, double u0, double v0);

	//! Constructor by default.
	Robot();

	//! Destructor.
	~Robot();

	//! Update the position of the robot on the picture (cv::Point2d imagePosition attribut) accordings to the two detected LEDs
	//! passed as parameters. 
	/*!
	\param p1 First detected LED coordinate
	\param p2 Second detected LED coordinate
	\return void
	*/
	void updatePosition(cv::Point p1, cv::Point p2);

	//! Compute the angle (in the trigonometric direction) between the position of the robot and the cv::Point
	//! desiredPosition attribut that the robot wants to reach. If the returned value is < 0, then the robot has to
	//! turn right, otherwise the robot has to turn left.
	/*!
	\param void
	\return The difference angle between the orientation of the robot and the point p
	*/
	double calculateRotation();

	//! Compute the distance between the position of the robot and the cv::Point desiredPosition attribut that the robot
	//! wants to reach.
	/*!
	\param void
	\return The distance angle between the orientation of the robot and the point p
	*/
	double calculateDistance();

	//! Send the command to the robot depending of the distance and the difference of orientation between the robot position
	//! and the cv::Point desiredPosition that the robot wants to reach. 
	/*!
	\param void
	\return void
	*/
	void sendCommandToRobot();

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

	//! Set the cv::Point desiredPosition attribut that the robot will try to reach.
	/*!
	\param p New point that the robot will try to reach
	\return void
	*/
	void setDesiredPosition(cv::Point p);

	//! Set the double H parameter (= distance between the ground and the fixed camera) that will be used to calculate
	//! the real position of the robot.
	/*!
	\param h Height of the camera
	\return void
	*/
	void setHeight(double h);

private:
	cv::Point3f realPosition;
	cv::Point2d imagePosition, desiredPosition;
	double H, alphaU, alphaV, u0, v0, angleOrientation;

	DCB dcb;
	BOOL result;
	HANDLE g_hPort;

	// Margin of error that are tolerate for the orientation of the robot
	const static int errorOrientation = 10;

	// Margin of error that are tolerate for the position of the robot
	const static int errorPosition = 5;
};
