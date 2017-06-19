#pragma once

#include <memory>
#include <thread>
#include <mutex> 

#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv2\opencv.hpp>

#include "robot.h"
#include "areaClassification.h"
#include "camera.h"
#include "interface2.h"

#include <QApplication>
#include <QImage>
#include <QLabel>

class MainInterface;

class Algo {
public:
	Algo();

	// Init
	void setInterface(MainInterface *);

	// Start the thread of the algo
	void start();
	// Terminate the thread
	void forceQuit();

	// When the interface is modified, this fucntion is called to update the value inside the Algo instance (<=> callback)
	void setDistanceAreaLight(int i);

	// Set resolution of the camera of the camera attribut
	void setResolution(int width, int height);

	// Set the parameters that will determine the displayed information on the picture
	void setDisplayPosition(bool val);
	void setDisplayOrientation(bool val);
	void setDisplayIdentification(bool val);
	void setDisplayKalman(bool val);

private:
	// Run the detection algorithm
	void run();

	bool started, finished, close;
	bool displayPosition, displayOrientation, displayIdentification, displayKalman;
	std::thread runThr;
	MainInterface * interf;
	Camera cam;
	std::mutex mtx;
};