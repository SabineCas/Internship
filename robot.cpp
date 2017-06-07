/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Robot
*/

#include "robot.h"

Robot::Robot(double H, double alphaU, double alphaV, double u0, double v0)
{
	this->imagePosition.x = -1;
	this->imagePosition.y = -1;
	this->realPosition.x = -1;
	this->realPosition.y = -1;
	this->realPosition.z = -1;
	this->u0 = u0;
	this->v0 = v0;
	this->alphaU = alphaU;
	this->alphaV = alphaV;
	this->H = H;
	this->angleOrientation = 0;
}

void Robot::updatePosition(cv::Point p1, cv::Point p2)
{
	// Update the image position
	imagePosition.x = (p1.x + p2.x) / 2;
	imagePosition.y = (p1.y + p2.y) / 2;
	//std::cout << "Image position : (" << imagePosition.x << ", " << imagePosition.y << std::endl;

	// Compute the real position in cm from the image position
	realPosition.x = float(this->H * (imagePosition.x - this->u0) / this->alphaU) * 100;
	realPosition.y = float(this->H * (imagePosition.y - this->v0) / this->alphaV) * 100;
	realPosition.z = 0;
	//std::cout << "Real position : (" << realPosition.x << ", " << realPosition.y << std::endl << std::endl;

	this->angleOrientation = cvFastArctan(p1.y - p2.y, p1.x - p2.x);
	//std::cout << "Orientation : " << this->angleOrientation << std::endl;
}

void Robot::displayImagePosition(cv::Mat image)
{
	// Display the position
	cv::circle(image, this->imagePosition, 5, cv::Scalar(0, 0, 255), -1, 8, 0);
	
	// Display the data
	cv::Point temp = this->imagePosition;

	temp.x = temp.x + 15;
	cv::putText(image, std::to_string(this->realPosition.x), temp, cv::FONT_HERSHEY_PLAIN,
		1, cv::Scalar(0, 0, 255), 2, 8, false);

	temp.y = temp.y + 20;
	cv::putText(image, std::to_string(this->realPosition.y), temp, cv::FONT_HERSHEY_PLAIN,
		1, cv::Scalar(0, 0, 255), 2, 8, false);

	temp.y = temp.y + 20;
	cv::putText(image, std::to_string(this->angleOrientation), temp, cv::FONT_HERSHEY_PLAIN,
		1, cv::Scalar(0, 0, 255), 2, 8, false);
}

cv::Point2d Robot::getImagePosition()
{
	return this->imagePosition;
}

cv::Point3f Robot::getRealPosition()
{
	return this->realPosition;
}




