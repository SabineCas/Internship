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
	this->desiredPosition.x = -1;
	this->desiredPosition.y = -1;
	this->realPosition.x = -1;
	this->realPosition.y = -1;
	this->realPosition.z = -1;
	this->u0 = u0;
	this->v0 = v0;
	this->alphaU = alphaU;
	this->alphaV = alphaV;
	this->H = H;
	this->angleOrientation = 0;

	// Initiate the communication with the MC
	serial = new QSerialPort();
	serial->setPortName("COM6");
	serial->setBaudRate(QSerialPort::Baud9600);
	serial->setDataBits(QSerialPort::Data8);
	serial->setParity(QSerialPort::NoParity);
	serial->setStopBits(QSerialPort::OneStop);
	serial->setFlowControl(QSerialPort::NoFlowControl);
	if (serial->open(QIODevice::WriteOnly)) {
		std::cout << "INIT COM6 OK" << std::endl;
	}
	else {
		std::cout << "INIT COM6 have encounted a pbl" << std::endl;
	}
}

Robot::Robot()
{
	this->imagePosition.x = -1;
	this->imagePosition.y = -1;
	this->desiredPosition.x = -1;
	this->desiredPosition.y = -1;
	this->realPosition.x = -1;
	this->realPosition.y = -1;
	this->realPosition.z = -1;
}

Robot::~Robot()
{
	//CloseHandle(this->g_hPort);
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

double Robot::calculateRotation()
{
	// Calculate the angle difference between the robot and the point p passed as parameter
	double angle = this->angleOrientation - cvFastArctan(this->desiredPosition.y - this->imagePosition.y,
		this->desiredPosition.x - this->imagePosition.x);

	// Chose the shorter rotation
	if (angle <= -180 && angle >= -360) {
		return(angle + 360);
	}
	else if (angle >= 180 && angle <= 360) {
		return(angle - 360);
	}
	return angle;
}

double Robot::calculateDistance()
{
	// Calculate the real distance
	double distanceX = (float(this->H * (this->desiredPosition.x - this->u0) / this->alphaU) * 100) - this->realPosition.x;
	double distanceY = (float(this->H * (this->desiredPosition.y - this->v0) / this->alphaV) * 100) - this->realPosition.y;
	//double distanceZ = 0;
	return sqrt(distanceX * distanceX + distanceY * distanceY);
}

void Robot::sendCommandToRobot()
{
	// Word of 4 bits
	char c = '0';

	if (this->serial->isOpen()) {
		if (this->calculateDistance() <= errorPosition) {
			// Send STOP
			c = 'S';
		}
		else {
			double angle = this->calculateRotation();
			if (abs(angle) <= errorOrientation) {
				// Send STRAIGHT/FORWARD
				c = 'F';
			}
			else if (abs(angle - 180) <= errorOrientation) {
				// Send BACK
				c = 'B';
			}
			else if (angle > 0) {
				// Send LEFT
				c = 'L';
			}
			else {
				// Send RIGHT
				c = 'R';
			}
		}
		if (c != '0') {
			// Send the emssage
			if (this->serial->write(&c)) {
				std::cout << "Msg send : " << c << std::endl;
			}
			else {
				std::cout << "Error : Msg not send" << std::endl;
			}
		}
	}
}

void Robot::displayImagePosition(cv::Mat image)
{
	// Display the position
	cv::circle(image, this->imagePosition, 5, cv::Scalar(255, 0, 0), -1, 8, 0);

	// Display the data
	cv::Point temp = this->imagePosition;

	temp.x = temp.x + 15;
	cv::putText(image, std::to_string(this->realPosition.x), temp, cv::FONT_HERSHEY_PLAIN,
		1, cv::Scalar(255, 0, 0), 1, 5, false);

	temp.y = temp.y + 20;
	cv::putText(image, std::to_string(this->realPosition.y), temp, cv::FONT_HERSHEY_PLAIN,
		1, cv::Scalar(255, 0, 0), 1, 5, false);

	temp.y = temp.y + 20;
	cv::putText(image, std::to_string(this->angleOrientation), temp, cv::FONT_HERSHEY_PLAIN,
		1, cv::Scalar(255, 0, 0), 1, 5, false);
}

void Robot::displayImageOrientation(cv::Mat image, cv::Point top)
{
	cv::arrowedLine(image, this->imagePosition, top, cv::Scalar(255, 0, 0), 3, 8, 0, 0.1);
}

cv::Point2d Robot::getImagePosition()
{
	return this->imagePosition;
}

cv::Point3f Robot::getRealPosition()
{
	return this->realPosition;
}

void Robot::setDesiredPosition(cv::Point p)
{
	this->desiredPosition = p;
}

void Robot::setHeight(double h)
{
	this->H = h;
}




