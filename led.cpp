#include "led.h"

LightArea::LightArea()
{
	this->visible = false;
	this->coord.x = -1;
	this->coord.y = -1;
	this->numArea = -1;
	this->LEDTimeON = 0;
	this->LEDTimeOFF = 0;
	this->identification = "UNKNOWN";
}

LightArea::LightArea(bool v, cv::Point c, int n, clock_t LEDTimeON, clock_t LEDTimeOFF, std::string id)
{
	this->visible = v;
	this->coord = c;
	this->numArea = n;
	this->LEDTimeON = 0;
	this->LEDTimeOFF = 0;
	this->identification = id;
}

bool LightArea::updateCoord(cv::Point p)
{
	if (cv::norm(this->coord - p) < distanceAreaLight) {
		this->coord = p;
		return(true);
	}
	return(false);
}

bool LightArea::areClose(cv::Point p)
{
	return(cv::norm(this->coord - p) < distanceAreaLight);
}

void LightArea::areaBlinkFreq(clock_t time, bool previousVisible)
{
	std::vector<clock_t> timeFrames;
	timeFrames.push_back(100);
	timeFrames.push_back(200);
	clock_t time1 = 100, time2 = 200;

	//// Update LEDTimeON and LEDTimeOFF
	//if (this->visible && !previousVisible) {
	//	LEDTimeON = time;
	//	this->identification = this->findMatchLEDTime(timeFrames, time, false);
	//}
	//else if (this->visible && previousVisible) {
	//	LEDTimeON += time;
	//	this->identification = this->findMatchLEDTime(timeFrames, time, false);
	//}
	//else if (!this->visible && previousVisible) {
	//	LEDTimeOFF = time;
	//	this->identification = this->findMatchLEDTime(timeFrames, time, true);
	//}
	//else {
	//	LEDTimeOFF += time;
	//	this->identification = this->findMatchLEDTime(timeFrames, time, true);
	//}

	//if (((LEDTimeON > time && LEDTimeON <= timeFrames[0]) || (LEDTimeOFF > time && LEDTimeOFF <= timeFrames[0])) && this->identification != "BOTTOM") {
	//	this->identification = "TOP";
	//}
	//else if ((LEDTimeON > time + timeFrames[0] && LEDTimeON <= timeFrames[1]) || (LEDTimeOFF > time + timeFrames[0] && LEDTimeOFF <= timeFrames[1])) {
	//	this->identification = "BOTTOM";
	//} else {
	//	this->identification = "UNKNOWN";
	//}

	if (this->visible && !previousVisible) {
		LEDTimeON = time;
		if (LEDTimeOFF <= time1 && LEDTimeOFF > 0) {
			this->identification = "TOP";
		}
		else if (LEDTimeOFF > time1 && LEDTimeOFF <= time2) {
			this->identification = "BOTTOM";
		}
		else {
			this->identification = "UNKNOWN";
		}
	}
	else if (this->visible && previousVisible) {
		LEDTimeON += time;
		if (LEDTimeOFF <= time1 && LEDTimeOFF > 0) {
			this->identification = "TOP";
		}
		else if (LEDTimeOFF > time1 && LEDTimeOFF <= time2) {
			this->identification = "BOTTOM";
		}
		else {
			this->identification = "UNKNOWN";
		}
	}
	else if (!this->visible && previousVisible) {
		LEDTimeOFF = time;
		if (LEDTimeON <= time1 && LEDTimeOFF > 0) {
			this->identification = "TOP";
		}
		else if (LEDTimeON > time1 && LEDTimeON <= time2) {
			this->identification = "BOTTOM";
		}
		else {
			this->identification = "UNKNOWN";
		}
	}
	else {
		LEDTimeOFF += time;
		if (LEDTimeON <= time1 && LEDTimeON > 0) {
			this->identification = "TOP";
		}
		else if (LEDTimeON > time1 && LEDTimeON <= time2) {
			this->identification = "BOTTOM";
		}
		else {
			this->identification = "UNKNOWN";
		}
	}
}

std::string LightArea::findMatchLEDTime(std::vector<clock_t> timeFrames, clock_t time, bool modeON)
{
	if (modeON) {
		for (std::vector<clock_t>::size_type i = 0; i < timeFrames.size(); i++) {
			if (this->LEDTimeON > time && this->LEDTimeON <= timeFrames[i]) {
				if (i == 0) {
					return("TOP");
				}
				else if (i == 1) {
					return("BOTTOM");
				}
			}
		}
	}
	else {
		for (std::vector<clock_t>::size_type i = 0; i < timeFrames.size(); i++) {
			if (this->LEDTimeOFF > time && this->LEDTimeOFF <= timeFrames[i]) {
				if (i == 0) {
					return("TOP");
				}
				else if (i == 1) {
					return("BOTTOM");
				}
			}
		}
	}
	return ("UNKNOWN");
}

bool LightArea::isContainedIn(std::vector<LightArea> vector)
{
	for (std::vector<LightArea>::size_type i = 0; i < vector.size(); i++) {
		if (this->getCoord() == vector[i].getCoord()) {
			return(true);
		}
	}
	return false;
}

int LightArea::findIn(std::vector<LightArea> vector)
{
	for (std::vector<LightArea>::size_type i = 0; i < vector.size(); i++) {
		if (this->getCoord() == vector[i].getCoord()) {
			return(i);
		}
	}
	return(-1);
}

bool LightArea::getVisible()
{
	return this->visible;
}

cv::Point LightArea::getCoord()
{
	return this->coord;
}

int LightArea::getNumArea()
{
	return this->numArea;
}

clock_t LightArea::getLEDTimeOFF()
{
	return this->LEDTimeOFF;
}

clock_t LightArea::getLEDTimeON()
{
	return this->LEDTimeON;
}

std::string LightArea::getIdentification()
{
	return this->identification;
}

void LightArea::setVisible(bool v)
{
	this->visible = v;
}

void LightArea::setCoord(int x, int y)
{
	this->coord.x = x;
	this->coord.y = y;
}

void LightArea::setNumArea(int n)
{
	this->numArea = n;
}

void LightArea::setLEDTimeOFF(clock_t t)
{
	this->LEDTimeOFF = t;
}

void LightArea::setLEDTimeON(clock_t t)
{
	this->LEDTimeON = t;
}

std::vector<LightArea> copyLAvector(std::vector<LightArea> vector1, std::vector<LightArea> vector2)
{
	vector2.clear();
	vector2.resize(0);
	for (std::vector<LightArea>::size_type i = 0; i < vector1.size(); i++) {
		vector2.push_back(vector1[i]);
	}
	return(vector2);
}
