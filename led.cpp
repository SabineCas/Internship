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
	this->LEDTimeON = LEDTimeON;
	this->LEDTimeOFF = LEDTimeOFF;
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
	clock_t time1(100), time2(200);
	timeFrames.push_back(time1);
	timeFrames.push_back(time2);

	// Update LEDTimeON and LEDTimeOFF
	if (this->visible && !previousVisible) {
		LEDTimeON = time;
		this->identification = this->findMatchLEDTime(timeFrames, time, true);
	}
	else if (this->visible && previousVisible) {
		LEDTimeON += time;
		this->identification = this->findMatchLEDTime(timeFrames, time, true);
	}
	else if (!this->visible && previousVisible) {
		LEDTimeOFF = time;
		this->identification = this->findMatchLEDTime(timeFrames, time, false);
	}
	else {
		LEDTimeOFF += time;
		this->identification = this->findMatchLEDTime(timeFrames, time, false);
	}
}

std::string LightArea::findMatchLEDTime(std::vector<clock_t> timeFrames, clock_t time, bool modeON)
{
	if (!modeON) {
		if (this->LEDTimeOFF > timeFrames[timeFrames.size() - 1]) {
			return("UNKNOWN");
		}
		for (std::vector<clock_t>::size_type i = 0; i < timeFrames.size(); i++) {
			if (i == 0) {
				if (this->LEDTimeON >= time && this->LEDTimeON <= timeFrames[i]) {
					return("TOP");
				}
			}
			else if (i == 1) {
				if (this->LEDTimeON >= timeFrames[i - 1] + time && this->LEDTimeON <= timeFrames[i]) {
					return("BOTTOM");
				}
			}
		}
	}
	else {
		if (this->LEDTimeON > timeFrames[timeFrames.size() - 1]) {
			return("UNKNOWN");
		}
		for (std::vector<clock_t>::size_type i = 0; i < timeFrames.size(); i++) {
			if (i == 0) {
				if (this->LEDTimeOFF >= time && this->LEDTimeOFF <= timeFrames[i]) {
					return("TOP");
				}
			}
			else if (i == 1) {
				if (this->LEDTimeOFF >= timeFrames[i - 1] + time && this->LEDTimeOFF <= timeFrames[i]) {
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
			return(int(i));
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

void updatePreviousToCurrent(std::vector<LightArea>& previousBlueVector, std::vector<LightArea>& blueVector, std::vector<LightArea>& finalBlueVector)
{
	bool found = false;
	// For each detected area in the previous frame, we add to the final vector every area that we can also detected in this frame
	// and update their value (visible, coordinate, ...) 
	for (std::vector<LightArea>::size_type i = 0; i < previousBlueVector.size(); i++) {
		found = false;
		for (std::vector<LightArea>::size_type j = 0; j < blueVector.size(); j++) {
			if (previousBlueVector[i].updateCoord(blueVector[j].getCoord())) {
				blueVector[j].setNumArea(previousBlueVector[i].getNumArea());
				blueVector[j].setVisible(true);
				blueVector[j].setLEDTimeON(previousBlueVector[i].getLEDTimeON());
				blueVector[j].setLEDTimeOFF(previousBlueVector[i].getLEDTimeOFF());
				if (!blueVector[j].isContainedIn(finalBlueVector)) {
					finalBlueVector.push_back(blueVector[j]);
				}
				found = true;
				break;
			}
		}
		if (!found) {
			finalBlueVector.push_back(LightArea(false, previousBlueVector[i].getCoord(), previousBlueVector[i].getNumArea(), previousBlueVector[i].getLEDTimeON(), previousBlueVector[i].getLEDTimeOFF(), previousBlueVector[i].getIdentification()));
		}
	}
}

void updateCurrentToPrevious(std::vector<LightArea>& previousBlueVector, std::vector<LightArea>& blueVector, std::vector<LightArea>& finalBlueVector)
{
	// For each detected area in this frame, we add to the final vector every new area from the previous frame
	for (std::vector<LightArea>::size_type i = 0; i < blueVector.size(); i++) {
		// The LightArea is not in the final vector, we add it, otherwise we just update it (but usually, it will not happen)
		if (!blueVector[i].isContainedIn(finalBlueVector)) {
			finalBlueVector.push_back(LightArea(true, blueVector[i].getCoord(), finalBlueVector.size(), blueVector[i].getLEDTimeON(), blueVector[i].getLEDTimeOFF(), blueVector[i].getIdentification()));
		}
	}
}

void updateIdentification(std::vector<LightArea>& previousBlueVector, std::vector<LightArea>& finalBlueVector, clock_t time)
{
	// Now that the final vector have the new detected and the previous updated areas, we can study the blinking frequency of every area
	// to determine if that matches with the LED frequency
	int indice;
	for (std::vector<LightArea>::size_type i = 0; i < finalBlueVector.size(); i++) {
		indice = finalBlueVector[i].findIn(previousBlueVector);
		if (indice == -1) {
			finalBlueVector[i].areaBlinkFreq(time, false);
		}
		else {
			finalBlueVector[i].areaBlinkFreq(time, previousBlueVector[indice].getVisible());
		}
	}
}
