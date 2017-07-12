/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Robot
*/

#pragma once
#include "camera.h"
#include "kalman.h"
#include <windows.h>

///! The Robot class stores all the information relative to the robot (position, orienation, control law, ...) and manage the control law.
class Robot {
public:
	///! Constructor that take as parameters the height of the camera and the parameters of the camera.
	/*!
	\param H Height of the camera
	\param alphaU The real size of the pixel on the X axis
	\param alphaV The real size of the pixel on the Y axis
	\param u0 The center of the picture in the image plan on the X axis
	\param v0 The center of the picture in the image plan on the Y axis
	\return void
	*/
	Robot(double H, double alphaU, double alphaV, double u0, double v0);

	///! Constructor by default.
	Robot();

	///! Update the position of the robot on the picture (cv::Point2d imagePosition attribut) accordings to the two detected LEDs
	/// passed as parameters. 
	/*!
	\param p1 First detected LED coordinate
	\param p2 Second detected LED coordinate
	\return void
	*/
	void updatePosition(cv::Point p1, cv::Point p2);

	///! Compute the angle (in the trigonometric direction) between the position of the robot and the cv::Point
	/// desiredPosition attribut that the robot wants to reach. If the returned value is < 0, then the robot has to
	/// turn right, otherwise the robot has to turn left.
	/*!
	\param void
	\return The difference angle between the orientation of the robot and the point p
	*/
	double calculateRotation();

	///! Compute the distance between the position of the robot and the cv::Point desiredPosition attribut that the robot
	/// wants to reach.
	/*!
	\param void
	\return The distance angle between the orientation of the robot and the point p
	*/
	double calculateDistance();

	///! Send the command to the robot depending of the distance and the difference of orientation between the robot position
	/// and the cv::Point desiredPosition that the robot wants to reach. 
	/*!
	\param void
	\return void
	*/
	void sendCommandToRobot();

	///! Send the command to the robot using the arrows of the keyboard or the button "Z", "Q", "S" and "D". This function
	/// is only working if the CheckBox Debug of the interface is set to True. 
	/*!
	\param void
	\return void
	*/
	void sendCommandToRobotDEBUG();

	///! Send the command to the robot depending of the difference of orientation between the robot position and the
	/// cv::Point desiredPosition that the robot wants to reach. 
	/*!
	\param dT Time between the previous frame and the current frame
	\return void
	*/
	void sendCommandToRobotArranged(clock_t dT);

	///! Send the command STOP to the robot.
	/*!
	\param void
	\return void
	*/
	void sendStop();

	///! Send the command BACKWARD to the robot.
	/*!
	\param debug True if the mode debug is ON, otherwise False
	\return void
	*/
	void sendBack(bool debug);

	///! Send the command RIGHT to the robot.
	/*!
	\param debug True if the mode debug is ON, otherwise False
	\return void
	*/
	void sendRight(bool debug);

	///! Send the command LEFT to the robot.
	/*!
	\param debug True if the mode debug is ON, otherwise False
	\return void
	*/
	void sendLeft(bool debug);

	///! Send the command FORWARD to the robot.
	/*!
	\param debug True if the mode debug is ON, otherwise False
	\return void
	*/
	void sendForward(bool debug);

	///! Convert the parameter int dec into a string that is the same value but in hexadecimal.
	/*!
	\param dec Decimal value to convert in hexadecimal value
	\return void
	*/
	std::string convertFromDecTo6BitsBinary(int dec);

	///! Display a filled circle on the current frame at the coordinate saved into the cv::Point2d imagePosition attribut.
	/*!
	\param image The current frame
	\return The current frame with the circle drawn
	*/
	void displayImagePosition(cv::Mat image);

	///! Display a filled circle on the current frame at the coordinate saved into the cv::Point2d desiredPosition attribut.
	/*!
	\param image The current frame
	\return The current frame with the circle drawn
	*/
	void displayDesiredPosition(cv::Mat image);

	///! Display an arrow on the currect frame at the coordinate saved into the cv::Point2d imagePosition attribut that represents the
	/// orientation of the robot.
	/*!
	\param image The current frame
	\param top Coordinate of the LEDs in front of the robot
	\return The current frame with the arrow drawn
	*/
	void displayImageOrientation(cv::Mat image, cv::Point top);

	///! Close properly the serial port communication before shutting down the program
	/*!
	\param void
	\return void
	*/
	void closeCom();

	///! Calculate the gain of the motors for the command forward and backward according to the distance between the 
	/// robot position and the wanted position. More the robot is far, more the gain is high. 
	/*!
	\param dist Distance between the robot position and the wanted position
	\return void
	*/
	int calculateGain(double dist);

	///! Return the image coordinate of the robot saved into the cv::Point2d imagePosition attribut.
	/*!
	\param void
	\return The cv::Point2d imagePosition attribut
	*/
	cv::Point2d getImagePosition();

	///! Return the coordinate of the robot into the world coordinate system saved into the cv::Point3f realPosition attribut.
	/*!
	\param void
	\return The cv::Point3f realPosition attribut
	*/
	cv::Point3f getRealPosition();

	///! Set the cv::Point desiredPosition attribut that the robot will try to reach.
	/*!
	\param p New point that the robot will try to reach
	\return void
	*/
	void setDesiredPosition(cv::Point p);

	///! Set the double H parameter (= distance between the ground and the fixed camera) that will be used to calculate
	/// the real position of the robot.
	/*!
	\param h Height of the camera
	\return void
	*/
	void setHeight(double h);

	///! Change the value of the boolean sendCommand parameter that represents the parameters that will allow to send
	/// the motion command to the robot or not.
	/*!
	\param c Value of the CheckBox to send command to the robot
	\return void
	*/
	void setSendCommand(bool c);

	///! Change the value of the int gainMotor1 (right motor) that represents the velocity of the right motor when a command
	/// is sent.
	/*!
	\param g Velocity of the motor to send command to the robot
	\return void
	*/
	void setGainMotor1(int g);

	///! Change the value of the int gainMotor2 (left motor) that represents the velocity of the left motor when a command
	/// is sent.
	/*!
	\param g Velocity of the motor to send command to the robot
	\return void
	*/
	void setGainMotor2(int g);

	///! Load the value of the motors gain for each command from the file "../data/MotorGain/gain.txt".
	/*!
	\param void
	\return void
	*/
	void loadGainFile();

private:
	cv::Point3f realPosition;
	cv::Point2d imagePosition, desiredPosition;
	double H, alphaU, alphaV, u0, v0, angleOrientation;
	bool sendCommand, debug;
	int gainMotor1, gainMotor2;

	int gainMotorRBackward, gainMotorLBackward, gainMotorRRight, gainMotorLRight,
		gainMotorRLeft, gainMotorLLeft, gainMotorRForward, gainMotorLForward;

	clock_t timeSTOP, timeBACK, timeRIGHT, timeLEFT;
	char previousCom;

	DCB dcb;
	BOOL result;
	HANDLE g_hPort;

	/// Margin of error that are tolerate for the orientation of the robot
	const static int errorOrientation = 20;

	/// Margin of error that are tolerate for the position of the robot
	const static int errorPosition = 10;
};
