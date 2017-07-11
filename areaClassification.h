/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Area classification
*/

#pragma once
#include "infraredLight.h"

//! The AreaClassification class determines which areas are blinking at the same frequency that the robot’s LEDs.
class AreaClassification {
public:
	//! Constructor by default.
	AreaClassification();

	//! Update the std::vector<infraredLight> infraredVector using the information from std::vector<infraredLight>
	// previousInfraredVector attribut. For each infraredLight contained into std::vector<infraredLight>
	// previousInfraredVector attribut but not into std::vector<infraredLight> infraredVector, the information is
	// updated and push into std::vector<infraredLight> finalInfraredVector attribut. 
	/*!
	\param void
	\return void
	*/
	void updateCurrentFromPrevious();

	//! Update the std::vector<infraredLight> previousInfraredVector using the information from std::vector<infraredLight>
	// infraredVector attribut. For each infraredLight contained into std::vector<infraredLight> infraredVector attribut,
	// it updates the std::vector<infraredLight> previousInfraredVector by comparing the distance between these two areas.
	// If there are close enough (cf const static int distanceAreaLight into the infraredLight file), the information is
	// updated and push into std::vector<infraredLight> finalInfraredVector attribut. 
	/*!
	\param void
	\return void
	*/
	void updatePreviousFromCurrent();

	//! Update the std::vector<infraredLight> finalInfraredVector to determine if the blinking frequency is matching with the
	// searched lights. For each infraredLight contained into std::vector<infraredLight> finalInfraredVector attribut, it
	// takes into consideration the time spent since the last iteration and determine if the identification is matching
	// with any searched infrared light frequency.
	/*!
	\param time Time spent since the last iteration.
	\return void
	*/
	void updateIdentification(clock_t time);

	//! Define the last known location of the searched LEDs (TOP and BOTTOM) using the information from the infraredVector and
	// previousInfraredVector attributs. 
	/*!
	\param void
	\return void
	*/
	void identifyLastKnownLocation();

	//! Display on the image the identification of the infraredLight conntained into the finalInfraredVector which have an
	// identification different from "UNKNOWN".
	/*!
	\param image Image where the identification will be display at the coordinates of the considered infraredLight.
	\return void
	*/
	void displayIdentification(cv::Mat image);

	//! Copy the std::vector<infraredLight> passed as a parameter into the std::vector<infraredLight> infraredVector
	//! (attribut of the class).
	/*!
	\param v std::vector<infraredLight> that will be copied into std::vector<infraredLight> infraredVector attribut.
	\return void
	*/
	void setInfraredVector(std::vector<infraredLight> v);

	//! Copy the std::vector<infraredLight> passed as a parameter into the std::vector<infraredLight> previousInfraredVector
	// (attribut of the class).
	/*!
	\param v std::vector<infraredLight> that will be copied into std::vector<infraredLight> previousInfraredVector attribut.
	\return void
	*/
	void setPreviousInfraredVector(std::vector<infraredLight> v);

	//! Clear the std::vector<infraredLight> finalInfraredVector attribut and resize it to 0.
	/*!
	\param void
	\return void
	*/
	void clearFinalInfraredVector();

	//! Return the std::vector<infraredLight> infraredVector attribut that represents the the detected area with the same
	// color range that the LEDs on the current frame.
	/*!
	\param void
	\return The std::vector<infraredLight> infraredVector attribut
	*/
	std::vector<infraredLight> getInfraredVector();

	//! Return the std::vector<infraredLight> previousInfraredVector attribut that represents the the detected area with the
	// same color range that the LEDs on the previous frame.
	/*!
	\param void
	\return The std::vector<infraredLight> previousInfraredVector attribut
	*/
	std::vector<infraredLight> getPreviousInfraredVector();

	//! Return the std::vector<infraredLight> finalInfraredVector attribut that represents the the detected area with the same
	// color range that the LEDs since the first frame after taking into account the previous and the current data.
	/*!
	\param void
	\return The std::vector<infraredLight> finalInfraredVector attribut
	*/
	std::vector<infraredLight> getFinalInfraredVector();

	//! Return the infraredLight lastKnownTOP attribut that represents the last known position of the LED on the top of the robot.
	/*!
	\param void
	\return The infraredLight lastKnownTOP attribut
	*/
	infraredLight getLastKnownTOP();

	//! Return the infraredLight lastKnownBOTTOM attribut that represents the last known position of the LED on the bottom
	// of the robot
	/*!
	\param void
	\return The infraredLight lastKnownBOTTOM attribut
	*/
	infraredLight getLastKnownBOTTOM();

private:
	std::vector<infraredLight> infraredVector, previousInfraredVector, finalInfraredVector;
	infraredLight lastKnownTOP, lastKnownBOTTOM;
};