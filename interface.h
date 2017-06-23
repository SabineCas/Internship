#pragma once

#include <stdio.h>
#include <functional>

#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv2\opencv.hpp>

#include "robot.h"
#include "areaClassification.h"

class WindowInterface
{
public:
	//! Constructor that take the number of robot and the maximal distance between two pixels belonging to the same area
	//! as parameters.
	/*!
	\param nbRobot Maximal number of robot that will appear in each frame
	\param valueDist Minimal distance between two pixels belonging to the same area
	\return void
	*/
	WindowInterface(int nbRobot, int valueDist);

	//! Callback function of the checkbox that will displayed the position of the robot on the current frame
	/*!
	\param state State of the button (True is it is check, otherwise False)
	\param data Pointer of the data that the callback function will use or modify
	\return void
	*/
	static void callbackButtonPosition(int state, void * data);

	//! Callback function of the checkbox that will displayed the orientation of the robot on the current frame
	/*!
	\param state State of the button (True is it is check, otherwise False)
	\param data Pointer of the data that the callback function will use or modify
	\return void
	*/
	static void callbackButtonOrientation(int state, void * data);

	//! Callback function of the checkbox that will displayed the identification of each area (other tha "UNKNOWN")
	//! on the current frame.
	/*!
	\param state State of the button (True is it is check, otherwise False)
	\param data Pointer of the data that the callback function will use or modify
	\return void
	*/
	static void callbackButtonIdentification(int state, void * data);

	//! Function that will start the interface and run the algorithm that will update the interface, but be careful
	//! this is a blocking function. You cannot taking back control of the algorithm.
	/*!
	\param void
	\return int Return 0 if everything went well, otherwise there was an issue to the execution of the interface
	*/
	int show();

private:
	int stateButtonPosition, stateButtonOrientation, stateButtonIdentification;
	int valueDistanceArea, factorResolution, previousFactorResolution, nbRobot;
	const char * titleWindows = "Graphical Interface - Spherical Robot For Child Care";
};

