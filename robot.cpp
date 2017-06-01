#include "robot.h"

Robot::Robot()
{
	this->imagePosition.x = -1;
	this->imagePosition.y = -1;
	this->realPosition.x = -1;
	this->realPosition.y = -1;
	this->realPosition.z = -1;
	this->blueLEDDetected = false;
}

cv::Point2d Robot::getImagePosition()
{
	return this->imagePosition;
}

cv::Point3f Robot::getRealPosition()
{
	return this->realPosition;
}

bool Robot::getBlueLEDDetected()
{
	return this->blueLEDDetected;
}

void Robot::setImagePosition(int x, int y)
{
	this->imagePosition.x = x;
	this->imagePosition.y = y;
}

void Robot::setRealPosition(float x, float y, float z)
{
	this->realPosition.x = x;
	this->realPosition.y = y;
	this->realPosition.z = z;
}

void Robot::setBlueLEDDetected(bool det)
{
	this->blueLEDDetected = det;
}


