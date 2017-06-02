#pragma once

#include <stdio.h>
#include <opencv2/opencv.hpp>

//! Minimal distance between two area infrared light.
const static int distanceAreaLight = 30;
//! Minimal size of a infraredLight to be take into account in the algorithm.
const static int minimumSizeAreaLight = 0;
//! Maximal size of a infraredLight to be take into account in the algorithm.
const static int maximumSizeAreaLight = 1000;

//! The infraredLight class identifies and stores every area in the picture that belongs to the same range color than the infrared LEDs.
class infraredLight {

public:
	//! Constructor by default.
	infraredLight();
	//! Constructor by copy.
	/*!
	\param il InfraredLight object to copy.
	*/
	//infraredLight(infraredLight& il);
	//! Constructor by copy.
	/*!
	\param v Boolean for the state (visible or not)
	\param c cv::Point for the coordinate of the infraredLight
	\param n Integer for the area number
	\param on Clock_t that represents the LED time ON
	\param off Clock_t that represents the LED time OFF
	\param s Integer for the size of the area
	\param id std::string for the identification of the LED
	\param tf std::vector<clock_t> for the time frames matching with the searched blinking frequency
	*/
	infraredLight(bool v, cv::Point c, int n, clock_t on, clock_t off, int s, std::string id);

	//! Determines if the point passed as a parameter is close to the current infraredLight coordinates.
	/*!
	\param p cv::Point to compare.
	\return True if the point passed as a prameter is close, otherwise false.
	*/
	bool areClose(cv::Point p);
	//! Update the LED off time, LED on time and the attribut "identification" of the current infraredLight using its state (visible or not),
	//! the LED off time and the LED on time.
	/*!
	\param time Period since the last measurement.
	\param previousVisible Previous state of the LED.
	\return void
	*/
	void areaBlinkFreq(clock_t time, bool previousVisible);
	//! Determines if the blinking frequency of the infraredLight matches with the blinking frequency of the searched LEDs.
	/*!
	\param time Period since the last measurement.
	\param previousVisible Previous state of the LED.
	\return void
	*/
	std::string findMatchLEDTime(std::vector<clock_t> timeFrames, clock_t time, bool modeON);
	//! Determines if the current infraredLight is contained into the vector passed as a parameter.
	/*!
	\param vector Vector of infraredLight in which the search will be done.
	\return True if the current infraredLight is contained into the vector passed as a parameter, otherwide false.
	*/
	bool isContainedIn(std::vector<infraredLight> vector);
	//! Return a vector of every index where the current infraredLight shares the same coordinates that the infraredLight contained into
	//! the vector passed as a parameter.
	/*!
	\param vector Vector of infraredLight in which the search will be done.
	\return Return a vector of every index where the current infraredLight shares the same coordinates.
	*/
	std::vector<int> findIn(std::vector<infraredLight> vector);
	//! Add time frame to the std::vector<clock_t> timeFrames attribut. It will be used to search every area that matching with this time frame.
	/*!
	\param time Blinking time (time when the area stay ON or time when the area stay OFF).
	\return void
	*/
	void addTimeFrame(clock_t time);

	bool getVisible();
	cv::Point getCoord();
	int getNumArea();
	int getSizeArea();
	clock_t getLEDTimeOFF();
	clock_t getLEDTimeON();
	std::string getIdentification();
	std::vector<clock_t> getTimeFrames();

	void setVisible(bool v);
	void setCoord(cv::Point p);
	void setNumArea(int n);
	//void setSizeArea(int size);
	void setLEDTimeOFF(clock_t t);
	void setLEDTimeON(clock_t t);

private:
	bool visible, previousVisible;
	cv::Point coord;
	int numArea, sizeArea;
	clock_t LEDTimeON, LEDTimeOFF;
	std::string identification;
};