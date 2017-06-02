#include "infraredLight.h"

infraredLight::infraredLight()
{
	this->visible = false;
	this->coord = cv::Point(-1, -1);
	this->numArea = -1;
	this->LEDTimeON = -1;
	this->LEDTimeOFF = -1;
	this->sizeArea = -1;
	this->identification = "UNKNOWN";
}

infraredLight::infraredLight(bool v, cv::Point c, int n, clock_t on, clock_t off, int s, std::string id)
{
	this->visible = v;
	this->coord = c;
	this->numArea = n;
	this->LEDTimeON = on;
	this->LEDTimeOFF = off;
	this->sizeArea = s;
	this->identification = id;
}

//infraredLight::infraredLight(infraredLight & il)
//{
//	this->visible = il.visible;
//	this->coord = il.coord;
//	this->numArea = il.numArea;
//	this->LEDTimeON = il.LEDTimeON;
//	this->LEDTimeOFF = il.LEDTimeOFF;
//	this->sizeArea = il.sizeArea;
//	this->identification = il.identification;
//	this->timeFrames = il.timeFrames;
//}

bool infraredLight::areClose(cv::Point p)
{
	return(cv::norm(this->coord - p) < distanceAreaLight);
}

void infraredLight::areaBlinkFreq(clock_t time, bool previousVisible)
{
	std::vector<clock_t> timeFrames;
	timeFrames.push_back(clock_t(100));
	timeFrames.push_back(clock_t(200));

	if (this->visible && !previousVisible) {
		this->LEDTimeON = time;
		this->identification = this->findMatchLEDTime(timeFrames, time, true);
	}
	else if (this->visible && previousVisible) {
		this->LEDTimeON += time;
		this->identification = this->findMatchLEDTime(timeFrames, time, true);
	}
	else if (!this->visible && previousVisible) {
		this->LEDTimeOFF = time;
		this->identification = this->findMatchLEDTime(timeFrames, time, false);
	}
	else {
		this->LEDTimeOFF += time;
		this->identification = this->findMatchLEDTime(timeFrames, time, false);
	}
}

std::string infraredLight::findMatchLEDTime(std::vector<clock_t> timeFrames, clock_t time, bool modeON)
{
	// In the case that the LED is ON
	if (!modeON) {
		if (!timeFrames.empty() && this->LEDTimeOFF > timeFrames[timeFrames.size() - 1]) {
			return("UNKNOWN");
		}
		for (std::vector<clock_t>::size_type i = 0; i < timeFrames.size(); i++) {
			if (i == 0) {
				if (this->LEDTimeON > time && this->LEDTimeON <= timeFrames[i]) {
					return("TOP");
				}
			}
			else if (i == 1) {
				if (this->LEDTimeON > timeFrames[i - 1] + time && this->LEDTimeON <= timeFrames[i]) {
					return("BOTTOM");
				}
			}
		}
	}
	// In the case that the LED is OFF
	else {
		if (!timeFrames.empty() && this->LEDTimeON > timeFrames[timeFrames.size() - 1]) {
			return("UNKNOWN");
		}
		for (std::vector<clock_t>::size_type i = 0; i < timeFrames.size(); i++) {
			if (i == 0) {
				if (this->LEDTimeOFF > time && this->LEDTimeOFF <= timeFrames[i]) {
					return("TOP");
				}
			}
			else if (i == 1) {
				if (this->LEDTimeOFF > timeFrames[i - 1] + time && this->LEDTimeOFF <= timeFrames[i]) {
					return("BOTTOM");
				}
			}
		}
	}
	return ("UNKNOWN");
}

bool infraredLight::isContainedIn(std::vector<infraredLight> vector)
{
	for (std::vector<infraredLight>::size_type i = 0; i < vector.size(); i++) {
		if (this->coord == vector[i].coord) {
			return(true);
		}
	}
	return false;
}

std::vector<int> infraredLight::findIn(std::vector<infraredLight> vector)
{
	std::vector<int> res;

	for (std::vector<infraredLight>::size_type i = 0; i < vector.size(); i++) {
		if (this->coord == vector[i].coord) {
			res.push_back(int(i));
		}
	}
	return(res);
}

void infraredLight::addTimeFrame(clock_t time)
{
	//this->timeFrames.push_back(time);
}

bool infraredLight::getVisible()
{
	return this->visible;
}

cv::Point infraredLight::getCoord()
{
	return this->coord;
}

int infraredLight::getNumArea()
{
	return this->numArea;
}

int infraredLight::getSizeArea()
{
	return this->sizeArea;
}

clock_t infraredLight::getLEDTimeOFF()
{
	return this->LEDTimeOFF;
}

clock_t infraredLight::getLEDTimeON()
{
	return this->LEDTimeON;
}

std::string infraredLight::getIdentification()
{
	return this->identification;
}

std::vector<clock_t> infraredLight::getTimeFrames()
{
	//return this->timeFrames;
	return(std::vector<clock_t>());
}

void infraredLight::setVisible(bool v)
{
	this->visible = v;
}

void infraredLight::setCoord(cv::Point p)
{
	this->coord = p;
}

void infraredLight::setNumArea(int n)
{
	this->numArea = n;
}

void infraredLight::setLEDTimeOFF(clock_t t)
{
	this->LEDTimeOFF = t;
}

void infraredLight::setLEDTimeON(clock_t t)
{
	this->LEDTimeON = t;
}
