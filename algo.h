/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Detection and command algorithm
*/

#pragma once
#include "robot.h"
#include "areaClassification.h"
#include "camera.h"
#include "interface2.h"
#include "infraredLight.h"

#include <QApplication>
#include <QImage>
#include <QLabel>

#include <thread>
#include <mutex> 
#include <iostream>

#include <opencv\highgui.h>
#include <opencv2\opencv.hpp>

class MainInterface;

//! The Algo class represents the class that will run the detection algorithm of the project.
class Algo {
public:
	//! Constructor by default
	Algo();

	//! Initialize the pointer to the interface that will update the algorithm or be updated by the algorithm.
	/*!
	\param mi The pointer MainInterface instance that will update the algorithm or be updated by the algorithm.
	\return void
	*/
	void setInterface(MainInterface * mi);

	//! Function that will start the thread of the algorithm which is different from the frame of the interface, but it will
	// not run the algorithm. It needs to call the private function "void run();" to do that.
	/*!
	\param void
	\return void
	*/
	void start();

	//! Function that will terminate the thread of the algorithm which is different from the frame of the interface.
	/*!
	\param void
	\return void
	*/
	void forceQuit();

	//! When the interface is modified, this function is called to update the value inside the Algo instance (<=> callback).
	/*!
	\param i The new maximal distance between two pixels that will belong to the same area
	\return void
	*/
	void setDistanceAreaLight(int i);

	//! Set the blinking frequency of the LEDs in the front of the robot.
	/*!
	\param i 
	\return void
	*/
	void setFreqLED1(int i);

	//! Set the blinking frequency of the LEDs in the back of the robot.
	/*!
	\param i 
	\return void
	*/
	void setFreqLED2(int i);

	//! Set resolution of the camera of the camera attribut. this function is called when the interface is modified. 
	/*!
	\param width The new width of the captured frame from the camera.
	\param height The new height of the captured frame from the camera.
	\return void
	*/
	void setResolution(int width, int height);

	//! Set the parameter that will determine if the position of the robot is displayed or not on the captured frame
	/*!
	\param val If the parameter val is True, the position of the robot is displayed on the frame, otherwise it is not displayed.
	\return void
	*/
	void setDisplayPosition(bool val);

	//! Set the parameter that will determine if the orientation of the robot is displayed or not on the captured frame
	/*!
	\param val If the parameter val is True, the orientation of the robot is displayed on the frame, otherwise it is not displayed.
	\return void
	*/
	void setDisplayOrientation(bool val);

	//! Set the parameter that will determine if the identification of each detected area (other than "UNKNOWN") is
	// displayed or not on the captured frame
	/*!
	\param val If the parameter val is True, the identification of each area is displayed on the frame, otherwise it is not displayed.
	\return void
	*/
	void setDisplayIdentification(bool val);

	//! Set the parameter that will determine if the estimate position of the robot from the Kalman filter is displayed
	// or not on the captured frame
	/*!
	\param val If the parameter val is True, the estimate position of the robot is displayed on the frame, otherwise it is not displayed.
	\return void
	*/
	void setDisplayKalman(bool val);

	//! Set the desired position point that the robot will try to reach.
	/*!
	\param x The position of the point on the X axis
	\param y The position of the point on the Y axis
	\return void
	*/
	void setDesiredPoint(int x, int y);

	//! Set the maximal number of robot that will appear in the frame
	/*!
	\param r Number of robots
	\return void
	*/
	void setNbRobot(int r);

	//! Set the distance between the ground and the fixed camera, that will be used to calculate the real position of
	// the robot.
	/*!
	\param h Height of the camera
	\return void
	*/
	void setHeight(int h);

	//! Set the attribut bool sendCommand that will allow the software to send another command tha STOP to the robot.
	/*!
	\param c Value of the CheckBox sendCommand in the interface
	\return void
	*/
	void sendCommand(bool c);

	//! Set the attribut gainMotor1 that will be used for setting the gain of the right motor in debug mode.
	/*!
	\param g Gain of the right motor
	\return void
	*/
	void setGainMotor1(int g);

	//! Set the attribut gainMotor1 that will be used for setting the gain of the left motor in debug mode.
	/*!
	\param g Gain of the left motor
	\return void
	*/
	void setGainMotor2(int g);

	//! Set the attribut debug that will allow the program to switch in debug mode or normal mode.
	/*!
	\param d Value of the CheckBox debug in the interface
	\return void
	*/
	void setDebug(bool d);

	//! Load the gain of each motor for each direction using the information contain t=in the text file
	// "../data/Motor.Gain/gain.txt".
	/*!
	\param d Value of the CheckBox loadGainFile in the interface
	\return void
	*/
	void loadGainFile(bool d);

private:
	//! Run the algorithm inside the std::thread runThr attribut that have been created. Warning : the public function
	// "void start();" have to be call before to initiate the thread of the algorithm.
	/*!
	\param void
	\return void
	*/
	void run();

	bool started, finished, close, debug;
	bool displayPosition, displayOrientation, displayIdentification, displayKalman;
	double height;
	int nbRobot;
	Camera cam;
	Robot robot;
	std::thread runThr;
	MainInterface * interf;
	std::mutex mtx;
};