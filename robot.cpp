#include "robot.h"

Robot::Robot()
{
	this->imagePosition.x = -1;
	this->imagePosition.y = -1;
	this->realPosition.x = -1;
	this->realPosition.y = -1;
	this->realPosition.z = -1;
}

void Robot::updatePosition(cv::Point p1, cv::Point p2)
{
	// Update the image position
	imagePosition.x = (p1.x + p2.x) / 2;
	imagePosition.y = (p1.y + p2.y) / 2;
	std::cout << "Image position : (" << imagePosition.x << ", " << imagePosition.y << std::endl;

	double H = 1.73, alphaU = 622.94114, alphaV = 626.18474, u0 = 303.41766, v0 = 232.79397;
	// Compute the real position from the image position
	realPosition.x = H * (imagePosition.x - u0) / alphaU;
	realPosition.y = H * (imagePosition.y - v0) / alphaV;
	realPosition.z = 0;
	std::cout << "Real position : (" << realPosition.x << ", " << realPosition.y << std::endl << std::endl;
}

cv::Mat Robot::displayImagePosition(cv::Mat image)
{
	circle(image, this->imagePosition, 5, cv::Scalar(0, 0, 255), -1, 8, 0);
	return image;
}

cv::Point2d Robot::getImagePosition()
{
	return this->imagePosition;
}

cv::Point3f Robot::getRealPosition()
{
	return this->realPosition;
}



