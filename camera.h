/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Camera calibration
*/

#pragma once

#define _CRT_SECURE_NO_DEPRECATE
#define DEBUG_CHESSBOARD

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "infraredLight.h"

// Value used for the threshold
const static int SENSITIVITY_VALUE = 30;
// Value used for the widening of the bounding box
const static int WIDE_BOUNDING_BOX_X = 20;

class Camera {
public:
	//! Constructor by default.
	/*!
	\param numDevice Number of the target device (0 is the device by default)
	*/
	Camera(int numDevice);

	//! Destructor by default.
	~Camera();

	//! Determines the intrinsic and distorsion parameters of the camera. If bool webcam parameter is true, the function
	//! will use the camera to capture 10 pictures with a chessboard that the user have to present to the camera and have to validate
	//! by pressing the space button of the keyboard. If bool webcam parameter is false, the function will use preset pictures to do
	//! the calibration. In both case, the function will save the intrinsec into the cv::Mat intrinsicParam attributs and the distorsion
	//! parameters into the cv::Mat distorsionParam.
	/*!
	\param webcam Boolean that determines if the camera will be use for the calibration or if the pictures in ../data/Calibration/will be used
	\return If something went wrong in the calibration, return -1, otherwise return 0
	*/
	int cameraCalib(bool webcam);

	//! Determines the correction maps to fix the geometrical distorsions of the camera. The function will save the result into the cv::Mat
	//! map1 and map2 attributs. 
	/*!
	\param H Height of the camera which is placed parallel to the ground (only in translation between the camera and the world coordinate system)
	\return void
	*/
	void cameraCorr(double H);

	//! Update the background that will be used for the backgorund subtraction using a KNN algorithm. 
	/*!
	\param image The current frame
	\param pKNN Type of algorithm to do the update of the background
	\return The image after the background subtraction
	*/
	cv::Mat updateBackground(cv::Mat image, cv::Ptr<cv::BackgroundSubtractor> pKNN);

	//! Improve the image with morphological processing.
	/*!
	\param image The current frame
	\return The image after the morphological processing
	*/
	cv::Mat improveBackSubtr(cv::Mat image);

	//! Detect areas where the color is in range between the cv::Scalar lower and the cv::Scalar upper color (in HSV)
	/*!
	\param image The current frame
	\param lower The lower limit of the color value in HSV
	\param upper The upper limit of the color value in HSV
	\return The binary image where every pixel in the range color takes the value 1, and all the other 0
	*/
	cv::Mat colorDetection(cv::Mat image, cv::Scalar lower, cv::Scalar upper);

	//! Detect areas where the color is in range between the cv::Scalar lower and the cv::Scalar upper color (in HSV)
	/*!
	\param image The current frame
	\param lower The lower limit of the color value in HSV
	\param upper The upper limit of the color value in HSV
	\return The std::vector<infraredLight> which saves every detected infraredLight with their characteristics (see the infraredLight class)
	*/
	std::vector<infraredLight> ledDetection(cv::Mat image, cv::Scalar lower, cv::Scalar upper);

	//! Use the Hough transformation to find circles inside a binary picture which will be save into the std::vector<cv::Vec3f> circles
	//! attribut. The circles find by the Hough transformation will be sort according to the vote in the accumulator in descending order.
	/*!
	\param subImage Binary picture
	\return void
	*/
	void circlesDetection(cv::Mat subImage);

	//! Evaluate the circle with the most vote in the accumulator accordings to the characteristics of the searched robot (size, ...)
	//! and the position of the detected markers. Then, if the position of the markers are inside or very near to circle, it returns
	//! the cv::Point that represents the center of the circle, otherwise it returns a cv::Point(-1, -1).
	/*!
	\param blueVector Vector with the potential position of the LEDs
	\return A cv::Point that represents the center of the circle recognized like coherent with all the parameters
	*/
	cv::Point coherenceCirclesMarkers(std::vector<cv::Point> blueVector);

	//! Display the circle contained in the first position of the std::vector<cv::Vec3f> circles attribut on the image passed as
	//! a parameter.
	/*!
	\param image The current frame
	\return The current frame with the circle drawn
	*/
	cv::Mat displayCircles(cv::Mat image);

	//! Widen the searching bounding box area (cv::Rect2d boundingBoxObs) attribut on each side with the value passed as a parameter.
	/*!
	\param value Value to add on every side of the bounding box
	\return void
	*/
	void wideringBoundingBox(int value);

	//! Return the cv::VideoCapture cap attribut that represents the capture instance of the camera
	/*!
	\param void
	\return The cv::VideoCapture cap attribut
	*/
	cv::VideoCapture getCap();

	//! Return the cv::Rect2d boundingBoxObs attribut that represents the searching bounding box area
	/*!
	\param void
	\return The cv::Rect2d boundingBoxObs attribut
	*/
	cv::Rect2d getBoundingBoxObs();

	//! Return the cv::Mat map1 attribut that represents the first map for the geometrical distorsion correction
	/*!
	\param void
	\return The cv::Mat map1 attribut
	*/
	cv::Mat getMap1();

	//! Return the cv::Mat map2 attribut that represents the first map for the geometrical distorsion correction
	/*!
	\param void
	\return The cv::Mat map2 attribut
	*/
	cv::Mat getMap2();

	//! Return the boolean detectedCircle attribut that represents the variable that save if a circle matches with the
	//! characteristics of the robot on the current frame.
	/*!
	\param void
	\return The boolean detectedCircle attribut
	*/
	bool getDetectedCircle();

	//! Set the cv::Rect2d boundingBoxObs attribut that represents the searching bounding box area.
	/*!
	\param minX The left-most pixel of the new bounding box
	\param maxX The right-most pixel of the new bounding box
	\param minY The up-most pixel of the new bounding box
	\param maxY The down-most pixel of the new bounding box
	\return void
	*/
	void setBoundingBoxObs(int minX, int maxX, int minY, int maxY);

	//! Set the boolean detectedCircle attribut that represents the variable that save if a circle matches with the
	//! characteristics of the robot on the current frame.
	/*!
	\param det The new value of the boolean detectedCircle attribut
	\return void
	*/
	void setDetectedCircle(bool det);
	

private:
	cv::VideoCapture cap;
	cv::Mat intrinsicParam, distortionParam, map1, map2;
	std::vector<cv::Vec3f> circles;
	cv::Rect2d boundingBoxObs;
	bool detectedCircle;
};
