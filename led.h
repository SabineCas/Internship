#pragma once

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace std;
const static int distanceAreaLight = 25;
const static int minimumSizeAreaLight = 10;
const static int maximumSizeAreaLight = 1000;

class LightArea {
public:
	LightArea();
	LightArea(bool v, cv::Point c, int n, clock_t LEDTimeON, clock_t LEDTimeOFF, std::string id);

	bool updateCoord(cv::Point p);
	bool areClose(cv::Point p);
	void areaBlinkFreq(clock_t time, bool previousVisible);
	std::string findMatchLEDTime(std::vector<clock_t> timeFrames, clock_t time, bool modeON);
	bool isContainedIn(std::vector<LightArea> vector);
	int findIn(std::vector<LightArea> vector);
	friend std::vector<LightArea> copyLAvector(std::vector<LightArea> vector1, std::vector<LightArea> vector2);
	friend void updatePreviousToCurrent(std::vector<LightArea>& previousBlueVector, std::vector<LightArea>& blueVector, std::vector<LightArea>& finalBlueVector);
	friend void updateCurrentToPrevious(std::vector<LightArea>& previousBlueVector, std::vector<LightArea>& blueVector, std::vector<LightArea>& finalBlueVector);
	friend void updateIdentification(std::vector<LightArea>& previousBlueVector, std::vector<LightArea>& finalBlueVector, clock_t time);

	bool getVisible();
	cv::Point getCoord();
	int getNumArea();
	clock_t getLEDTimeOFF();
	clock_t getLEDTimeON();
	std::string getIdentification();

	void setVisible(bool v);
	void setCoord(int x, int y);
	void setNumArea(int n);
	void setLEDTimeOFF(clock_t t);
	void setLEDTimeON(clock_t t);


private:
	bool visible;
	cv::Point coord;
	int numArea;
	clock_t LEDTimeON, LEDTimeOFF;
	std::string identification;
};