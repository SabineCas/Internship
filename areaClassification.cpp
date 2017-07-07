#include "areaClassification.h"

AreaClassification::AreaClassification()
{
	this->infraredVector = std::vector<infraredLight>();
	this->previousInfraredVector = std::vector<infraredLight>();
	this->finalInfraredVector = std::vector<infraredLight>();
}

void AreaClassification::updateCurrentFromPrevious()
{
	bool found;
	// For each detected area in the previous frame, we add to the final vector every area that we can also detected in this frame
	// and update their value (visible, coordinate, ...) 
	for (std::vector<infraredLight>::size_type i = 0; i < this->previousInfraredVector.size(); i++) {
		found = false;
		for (std::vector<infraredLight>::size_type j = 0; j < this->infraredVector.size(); j++) {
			if (this->previousInfraredVector[i].areClose(infraredVector[j].getCoord())) {
				// Update the information data before the push
				previousInfraredVector[i].setCoord(infraredVector[j].getCoord());
				this->infraredVector[j].setNumArea(this->previousInfraredVector[i].getNumArea());
				//this->infraredVector[j].setSizeArea(this->previousInfraredVector[i].getSizeArea());
				this->infraredVector[j].setLEDTimeON(this->previousInfraredVector[i].getLEDTimeON());
				this->infraredVector[j].setLEDTimeOFF(this->previousInfraredVector[i].getLEDTimeOFF());
				this->infraredVector[j].setVisible(true);
				// Push into the finalInfraredVector 
				if (!this->infraredVector[j].isContainedIn(this->finalInfraredVector)) {
					this->finalInfraredVector.push_back(this->infraredVector[j]);
				}
				found = true;
				break;
			}
		}
		if (!found) {
			this->finalInfraredVector.push_back(infraredLight(false, this->previousInfraredVector[i].getCoord(),
				this->previousInfraredVector[i].getNumArea(), this->previousInfraredVector[i].getLEDTimeON(),
				this->previousInfraredVector[i].getLEDTimeOFF(), this->previousInfraredVector[i].getSizeArea(),
				this->previousInfraredVector[i].getIdentification()));
		}
	}
}

void AreaClassification::updatePreviousFromCurrent()
{
	// For each detected area in this frame, we add to the final vector every new area from the previous frame
	for (std::vector<infraredLight>::size_type i = 0; i < this->infraredVector.size(); i++) {
		// The infraredLight is not in the final vector, we just add it, otherwise we just update it
		// (but usually, it will not happen if the updateCurrentFromPrevious function is called before)
		if (!this->infraredVector[i].isContainedIn(this->finalInfraredVector)) {
			this->finalInfraredVector.push_back(infraredLight(true, this->infraredVector[i].getCoord(),
				int(this->finalInfraredVector.size()), this->infraredVector[i].getLEDTimeON(),
				this->infraredVector[i].getLEDTimeOFF(), this->infraredVector[i].getSizeArea(),
				this->infraredVector[i].getIdentification()));
		}
	}
}

void AreaClassification::updateIdentification(clock_t time)
{
	// Now that the final vector have the new detected and the previous updated areas, we can study the blinking frequency of every area
	// to determine if that matches with the LED frequency
	std::vector<int> index;
	for (std::vector<infraredLight>::size_type i = 0; i < this->finalInfraredVector.size(); i++) {
		index = this->finalInfraredVector[i].findIn(this->previousInfraredVector);
		if (index.empty()) {
			this->finalInfraredVector[i].areaBlinkFreq(time, false);
		}
		else {
			infraredLight temp;
			int indice = 0;
			// TODO : chose the best area when there is several area with the same state ("TOP" or "BOTTOM")
			// Will depend of the number of robot
			/*for (std::vector<int>::size_type j = 0; j < index.size(); j++) {
				if (temp.getSizeArea() < this->finalInfraredVector[i].getSizeArea()) {
					temp = this->finalInfraredVector[i];
					indice = j;
				}
			}*/
			this->finalInfraredVector[i].areaBlinkFreq(time, this->previousInfraredVector[index[indice]].getVisible());
		}
	}
}

void AreaClassification::identifyLastKnownLocation()
{
	this->lastKnownTOP = infraredLight();
	this->lastKnownBOTTOM = infraredLight();
	// Identify the last known location of the different LEDs using the information from the current and previous LEDs vector
	for (std::vector<infraredLight>::size_type i = 0; i < this->finalInfraredVector.size(); i++) {
		if (this->finalInfraredVector[i].getIdentification() != "UNKNOWN") {
			if (!this->finalInfraredVector[i].findIn(this->previousInfraredVector).empty()) {
				std::string id = this->finalInfraredVector[i].getIdentification();
				std::string prevId = this->previousInfraredVector[this->finalInfraredVector[i].findIn(this->previousInfraredVector)[0]].getIdentification();
				if (id == "TOP" && prevId == "TOP") {
					this->lastKnownTOP = this->finalInfraredVector[i];
				}
				else if (id == "BOTTOM" && prevId == "BOTTOM") {
					this->lastKnownBOTTOM = this->finalInfraredVector[i];
				}
				// Make the application less robust but accept to suppose that the LED could be recognize at these coordinate
				/*else if (lastKnownTOP.getCoord() == cv::Point(-1, -1)) {
					if (id == "TOP") {
						this->lastKnownTOP = this->finalInfraredVector[i];
					} else if (id == "BOTTOM") {
						this->lastKnownBOTTOM = this->finalInfraredVector[i];
					}
				}*/
			}
		}
	}
}

void AreaClassification::displayIdentification(cv::Mat image)
{
	cv::putText(image, this->lastKnownBOTTOM.getIdentification(), this->lastKnownBOTTOM.getCoord(),
		cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, 8, false);
	cv::putText(image, this->lastKnownTOP.getIdentification(), this->lastKnownTOP.getCoord(),
		cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, 8, false);

	for (std::vector<infraredLight>::size_type i = 0; i < this->finalInfraredVector.size(); i++) {
		if (this->finalInfraredVector[i].getIdentification() == "UNKNOWN") {
			cv::putText(image, this->finalInfraredVector[i].getIdentification(), this->finalInfraredVector[i].getCoord(),
				cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, 8, false);
		}
	}
}

void AreaClassification::setInfraredVector(std::vector<infraredLight> v)
{
	this->infraredVector.clear();
	this->infraredVector.resize(0);
	for (std::vector<infraredLight>::size_type i = 0; i < v.size(); i++) {
		this->infraredVector.push_back(v[i]);
	}
}

void AreaClassification::setPreviousInfraredVector(std::vector<infraredLight> v)
{
	this->previousInfraredVector.clear();
	this->previousInfraredVector.resize(0);
	for (std::vector<infraredLight>::size_type i = 0; i < v.size(); i++) {
		this->previousInfraredVector.push_back(v[i]);
	}
}

void AreaClassification::clearFinalInfraredVector()
{
	this->finalInfraredVector.clear();
	this->finalInfraredVector.resize(0);
}

std::vector<infraredLight> AreaClassification::getInfraredVector()
{
	return this->infraredVector;
}

std::vector<infraredLight> AreaClassification::getPreviousInfraredVector()
{
	return this->previousInfraredVector;
}

std::vector<infraredLight> AreaClassification::getFinalInfraredVector()
{
	return this->finalInfraredVector;
}

infraredLight AreaClassification::getLastKnownTOP()
{
	return this->lastKnownTOP;
}

infraredLight AreaClassification::getLastKnownBOTTOM()
{
	return this->lastKnownBOTTOM;
}
