/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Infrared light
*/

#pragma once
#include <opencv2/opencv.hpp>

//! Minimal distance between two area infrared light.
extern int distanceAreaLight;
extern int timeLED1;
extern int timeLED2;

//! Minimal size of a infraredLight to be take into account in the algorithm.
const static int minimumSizeAreaLight = 0;
//! Maximal size of a infraredLight to be take into account in the algorithm.
const static int maximumSizeAreaLight = 1000;

//! The infraredLight class identifies and stores every area in the picture that belongs to the same range color than the infrared LEDs.
class infraredLight {

public:
	//! Constructor by default.
	infraredLight();

	//! Constructor by copy setting each attributs passed as parameters in this function.
	/*!
	\param v Boolean for the state (visible or not)
	\param c cv::Point for the coordinate of the infraredLight
	\param n Integer for the area number
	\param on Clock_t that represents the LED time ON
	\param off Clock_t that represents the LED time OFF
	\param s Integer for the size of the area
	\param id std::string for the identification of the LED
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

	//! Return the boolean visible attribut that represents the state of the LED (ON or OFF)
	/*!
	\param void
	\return The boolean visible attribut
	*/
	bool getVisible();

	//! Return the cv::Point coord attribut that represents the coordinate of the gravity center of the area inside the picture
	/*!
	\param void
	\return The cv::Point coord attribut
	*/
	cv::Point getCoord();

	//! Return the int numArea attribut that represents the unique identification number of the area
	/*!
	\param void
	\return The int numArea attribut
	*/
	int getNumArea();

	//! Return the int sizeArea attribut that represents the size of area inside the picture
	/*!
	\param void
	\return The int sizeArea attribut
	*/
	int getSizeArea();

	//! Return the clock_t LEDTimeOFF attribut that represents the time when the area was OFF since the last state ON
	/*!
	\param void
	\return The clock_t LEDTimeOFF attribut
	*/
	clock_t getLEDTimeOFF();

	//! Return the clock_t LEDTimeON attribut that represents the time when the area was ON since the last state OFF
	/*!
	\param void
	\return The clock_t LEDTimeON attribut
	*/
	clock_t getLEDTimeON();

	//! Return the std::string identification attribut that represents the identification "TOP" or "BOTTOM" or "UNKNOWN"
	/*!
	\param void
	\return The std::string identification attribut
	*/
	std::string getIdentification();

	//! Set the bool visible attribut that represents the state of the LED (ON or OFF)
	/*!
	\param bool New state of the LED
	\return void
	*/
	void setVisible(bool v);

	//! Set the cv::Point coord attribut that represents the coordinate of the gravity center of the area inside the picture
	/*!
	\param cv::Point New coordinate of the area inside the picture
	\return void
	*/
	void setCoord(cv::Point p);

	//! Set the int numArea attribut that represents the unique identification number of the area
	/*!
	\param int New unique identification number of the area
	\return void
	*/
	void setNumArea(int n);

	//! Set the clock_t LEDTimeOFF attribut that represents the time when the area was OFF since the last state ON
	/*!
	\param int New time OFF of the area
	\return void
	*/
	void setLEDTimeOFF(clock_t t);

	//! Set the clock_t LEDTimeON attribut that represents the time when the area was ON since the last state OFF
	/*!
	\param int New time ON of the area
	\return void
	*/
	void setLEDTimeON(clock_t t);

private:
	bool visible, previousVisible;
	cv::Point coord;
	int numArea, sizeArea;
	clock_t LEDTimeON, LEDTimeOFF;
	std::string identification;
};