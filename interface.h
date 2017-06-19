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
	WindowInterface(int nbRobot, int valueDist);

	static void callbackButtonPosition(int state, void * data);
	static void callbackButtonOrientation(int state, void * data);
	static void callbackButtonIdentification(int state, void * data);
	void show();

private:
	int stateButtonPosition, stateButtonOrientation, stateButtonIdentification;
	int valueDistanceArea, factorResolution, previousFactorResolution, nbRobot;
	const char * titleWindows = "Graphical Interface - Spherical Robot For Child Care";
};

