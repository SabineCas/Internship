#pragma once

#include <memory>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv2\opencv.hpp>

#include "robot.h"
#include "areaClassification.h"

#include <QApplication>
#include <QImage>
#include <QLabel>

class Interface;

class Algo {
public:
	Algo();
	void start();
	void forceQuit();
	void setInterface(Interface*);
	void setValue(int i);
	int getValue();

private:
	class Impl;
	std::shared_ptr<Impl> impl;
};