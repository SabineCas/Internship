/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Robot
*/

#include "robot.h"
#include <stdio.h>
#include <tchar.h>
#include <bitset>

Robot::Robot(double H, double alphaU, double alphaV, double u0, double v0)
{
	// Data linked to the camera
	this->u0 = u0;
	this->v0 = v0;
	this->alphaU = alphaU;
	this->alphaV = alphaV;
	this->H = H;

	// Information about the robot
	this->imagePosition.x = 0;
	this->imagePosition.y = 0;
	this->desiredPosition.x = 0;
	this->desiredPosition.y = 0;
	this->realPosition.x = -1;
	this->realPosition.y = -1;
	this->realPosition.z = -1;
	this->angleOrientation = 0;

	// Connection Xbee
	this->sendCommand = false;
	this->result = false;

	// Timers for commands
	this->timeSTOP = 0;
	this->timeBACK = 0;
	this->timeRIGHT = 0;
	this->timeLEFT = 0;

	// Motor gain
	this->gainMotor1 = 14;
	this->gainMotor2 = 14;
	this->loadGainFile();
	
	// Initiate the communication with the MC
	this->g_hPort = CreateFile(_T("COM6"), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_NEW, 0, NULL);
	GetCommState(this->g_hPort, &dcb);
	dcb.BaudRate = 9600;
	dcb.fParity = FALSE;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	this->result = SetCommState(this->g_hPort, &dcb);
	if (this->result) {
		std::cout << "INIT COM6 OK" << std::endl;
	}
	else {
		std::cout << "INIT COM6 have encounted a pbl" << std::endl;
	}
}

Robot::Robot()
{
	// Information about the robot
	this->imagePosition.x = 0;
	this->imagePosition.y = 0;
	this->desiredPosition.x = 0;
	this->desiredPosition.y = 0;
	this->realPosition.x = -1;
	this->realPosition.y = -1;
	this->realPosition.z = -1;

	// Connection Xbee
	this->sendCommand = false;
	this->result = false;

	// Timers for commands
	this->timeSTOP = 0;
	this->timeBACK = 0;
	this->timeRIGHT = 0;
	this->timeLEFT = 0;

	// Motor gain
	this->gainMotor1 = 14;
	this->gainMotor2 = 14;
	this->loadGainFile();
}

void Robot::updatePosition(cv::Point p1, cv::Point p2)
{
	// Update the image position
	imagePosition.x = (p1.x + p2.x) / 2;
	imagePosition.y = (p1.y + p2.y) / 2;

	// Compute the real position in cm from the image position
	realPosition.x = float(this->H * (imagePosition.x - this->u0) / this->alphaU) * 100;
	realPosition.y = float(this->H * (imagePosition.y - this->v0) / this->alphaV) * 100;
	realPosition.z = 0;

	// Compute the orientation of the robot in the camera coordinate system
	this->angleOrientation = cvFastArctan(p1.y - p2.y, p1.x - p2.x);
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
	// Meaning of the data
	unsigned char c = '0';

	if (result && sendCommand) {
		double distance = this->calculateDistance();

		if (distance <= errorPosition || this->desiredPosition.x < 0) {
			// Send STOP
			this->sendStop();
		}
		else {
			double angle = this->calculateRotation();
			if (abs(angle) <= errorOrientation) {
				// STRAIGHT/FORWARD command
				c = 'f';
				this->sendForward(false);
			}
			else if (abs(angle - 180) <= errorOrientation) {
				// BACK command
				c = 'b';
				this->sendBack(false);
			}
			else if ((angle >= 0 && angle <= 90) || (angle >= -180 && angle <= -90)) {
				// LEFT command
				c = 'l';
				this->sendLeft(false);
			}
			else {
				// RIGHT command
				c = 'r';
				this->sendRight(false);
			}
		}
	}
}

void Robot::sendCommandToRobotDEBUG()
{
	unsigned char c = '0';

	if (result) {
		int key = cv::waitKey(10);
		if (key == 38 || key == 122) {
			//Forward
			c = 'f';
			this->sendForward(true);
		}
		else if (key == 37 || key == 113) {
			// Left
			c = 'l';
			this->sendLeft(true);
		}
		else if (key == 39 || key == 100) {
			// Right
			c = 'r';
			this->sendRight(true);
		}
		else if (key == 40 || key == 115) {
			// Backward
			c = 'b';
			this->sendBack(true);
		}
		else if (key == 32) {
			this->sendStop();
		}
	}
}

void Robot::sendCommandToRobotArranged(clock_t dT)
{
	// Timer for each command and time between commands
	int timeBetweenCom = 1500, timeRIGHTCom = 100, timeLEFTCom = 100, timeBACKCom = 800;

	// Meaning of the data
	unsigned char c = '0';

	if (result && sendCommand) {
		// Choose the command to send depending of the position and orientation of the robot
		if (this->calculateDistance() <= errorPosition || this->desiredPosition.x < 0) {
			// Send STOP
			c = 's';
		}
		else {
			if (abs(this->calculateRotation() - 180) <= errorOrientation ||
				abs(this->calculateRotation() + 180) <= errorOrientation) {
				// Go backward for timeBACKCom milliseconds max
				c = 'b';
			}
			else if(abs(this->calculateRotation() - 180) <= 180) {
				// Turn right for timeRIGHTCom milliseconds max
				c = 'l';
			}
			else {
				// Turn right for timeRIGHTCom milliseconds max
				c = 'r';
			}
		}

		// Adapting the command depending of the previous command and the rest time between two command
		// If the current command is STOP
		if (c == 's') {
			if (this->previousCom != 's') {
				this->timeSTOP = 0;
			}
			else {
				this->timeSTOP += dT;
			}
		}
		// If the current command is BACK
		else if (c == 'b' && this->previousCom == 'b') {
			if (this->timeBACK <= timeBACKCom) {
				this->timeBACK += dT;
			}
			else {
				c = 's';
				this->timeSTOP = 0;
			}
		}
		else if (c == 'b' && (this->previousCom == 'r' || this->previousCom == 'l')) {
			c = 's';
			this->timeSTOP = 0;
		}
		else if (c == 'b' && this->previousCom == 's') {
			if (this->timeSTOP >= timeBetweenCom) {
				this->timeBACK = 0;
			}
			else {
				c = 's';
				this->timeSTOP += dT;
			}
		}
		// If the current command is RIGHT
		else if (c == 'r' && this->previousCom == 'r') {
			if (this->timeRIGHT <= timeRIGHTCom) {
				this->timeRIGHT += dT;
			}
			else {
				c = 's';
				this->timeSTOP = 0;
			}
		}
		else if (c == 'r' && (this->previousCom == 'b' || this->previousCom == 'l')) {
			c = 's';
			this->timeSTOP = 0;
		}
		else if (c == 'r' && this->previousCom == 's') {
			if (this->timeSTOP >= timeBetweenCom) {
				this->timeRIGHT = 0;
			}
			else {
				c = 's';
				this->timeSTOP += dT;
			}
		}
		// If the current command is LEFT
		else if (c == 'l' && this->previousCom == 'l') {
			if (this->timeLEFT <= timeLEFTCom) {
				this->timeLEFT += dT;
			}
			else {
				c = 's';
				this->timeSTOP = 0;
			}
		}
		else if (c == 'l' && (this->previousCom == 'b' || this->previousCom == 'r')) {
			c = 's';
			this->timeSTOP = 0;
		}
		else if (c == 'l' && this->previousCom == 's') {
			if (this->timeSTOP >= timeBetweenCom) {
				this->timeLEFT = 0;
			}
			else {
				c = 's';
				this->timeSTOP += dT;
			}
		}

		// Send the real command
		if (c == 'b') {
			this->sendBack(false);
		}
		else if (c == 'r') {
			this->sendRight(false);
		}
		else if (c == 'l') {
			this->sendLeft(false);
		}
		else {
			this->sendStop();
		}

		// Update the previous command
		this->previousCom = c;
	}
}

std::string Robot::convertFromDecTo6BitsBinary(int dec)
{
	std::string valueBin;
	// Binary value
	while (dec != 0) {
		valueBin.append(std::to_string(dec % 2));
		dec /= 2;
	}
	// Add 0 at the end while the binary value has less than 6 numbers
	while (valueBin.size() < 6) {
		valueBin.append("0");
	}
	return valueBin;
}

void Robot::displayImagePosition(cv::Mat image)
{
	// Display the position
	cv::circle(image, this->imagePosition, 5, cv::Scalar(255, 0, 0), -1, 8, 0);

	// Display the real position and orientation fo the robot
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

void Robot::displayDesiredPosition(cv::Mat image)
{
	// Display the wanted position that the robot has to reach
	cv::circle(image, this->desiredPosition, 5, cv::Scalar(0, 0, 255), -1, 8, 0);
}

void Robot::displayImageOrientation(cv::Mat image, cv::Point top)
{
	// Display the orientation
	cv::arrowedLine(image, this->imagePosition, top, cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);
}

void Robot::closeCom()
{
	// Stop the robot before shutting down the program
	unsigned char c1 = (unsigned char)0xA0;
	unsigned char c2 = (unsigned char)0x60;
	DWORD dwWriteBytes;
	WriteFile(g_hPort, &c1, 1, &dwWriteBytes, NULL);
	Sleep(10);
	WriteFile(g_hPort, &c2, 1, &dwWriteBytes, NULL);
	Sleep(1000);

	if (GetCommState(g_hPort, &dcb)) {
		// Close file
		DeleteFile(_T("COM6"));
		// Close the COM port
		CloseHandle(g_hPort);
	}
}

int Robot::calculateGain(double dist)
{
	// When the robot is at 50 cm from the robot, the speed is the most higher
	double coef = 50.0 / 31.0;
	if (dist > 50) {
		return(31);
	}
	else {
		return(int(dist / coef));
	}
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

void Robot::setSendCommand(bool c)
{
	this->sendCommand = c;
	if (!c) {
		// Stop the robot before shutting down the program
		unsigned char c1 = (unsigned char)0xA0;
		unsigned char c2 = (unsigned char)0x60;
		DWORD dwWriteBytes;
		WriteFile(g_hPort, &c1, 1, &dwWriteBytes, NULL);
		Sleep(10);
		WriteFile(g_hPort, &c2, 1, &dwWriteBytes, NULL);
	}
}

void Robot::setGainMotor1(int g)
{
	this->gainMotor1 = g;
}

void Robot::setGainMotor2(int g)
{
	this->gainMotor2 = g;
}

void Robot::loadGainFile()
{
	std::ifstream myfile;
	myfile.open("../data/MotorGain/gain.txt");
	if (myfile.is_open())
	{
		std::string temp;
		while (temp != "Backward") {
			std::getline(myfile, temp);
		}
		std::getline(myfile, temp);
		this->gainMotorRBackward = stoi(temp);
		std::getline(myfile, temp);
		this->gainMotorLBackward = stoi(temp);

		while (temp != "Right") {
			std::getline(myfile, temp);
		}
		std::getline(myfile, temp);
		this->gainMotorRRight = stoi(temp);
		std::getline(myfile, temp);
		this->gainMotorLRight = stoi(temp);

		while (temp != "Left") {
			std::getline(myfile, temp);
		}
		std::getline(myfile, temp);
		this->gainMotorRLeft = stoi(temp);
		std::getline(myfile, temp);
		this->gainMotorLLeft = stoi(temp);

		while (temp != "Forward") {
			std::getline(myfile, temp);
		}
		std::getline(myfile, temp);
		this->gainMotorRForward = stoi(temp);
		std::getline(myfile, temp);
		this->gainMotorLForward = stoi(temp);
	}
	myfile.close();
}

void Robot::sendStop()
{
	// Stop the robot before shutting down the program
	unsigned char c1 = (unsigned char)0xA0;
	unsigned char c2 = (unsigned char)0x60;

	DWORD dwWriteBytes;
	WriteFile(g_hPort, &c1, 1, &dwWriteBytes, NULL);
	Sleep(10);
	WriteFile(g_hPort, &c2, 1, &dwWriteBytes, NULL);
	std::cout << "STOP" << std::endl;
}

void Robot::sendBack(bool debug)
{
	int gainMotorR, gainMotorL;

	if (!debug) {
		gainMotorR = this->gainMotorRBackward;
		gainMotorL = this->gainMotorLBackward;
	}
	else {
		gainMotorR = this->gainMotor1;
		gainMotorL = this->gainMotor2;
	}

	int valueR = 31 - gainMotorR;
	int valueL = 31 - gainMotorL;

	// Conversion from decimal to binary
	std::string valueBin1 = convertFromDecTo6BitsBinary(valueR);
	std::string valueBin2 = convertFromDecTo6BitsBinary(valueL);

	// Add the "command" part
	valueBin1.append("01");
	valueBin2.append("10");

	// Reverse the binary value
	std::reverse(valueBin1.begin(), valueBin1.end());
	std::reverse(valueBin2.begin(), valueBin2.end());

	// Convert to the hexadecimal value
	std::bitset<8> valueHex1(valueBin1);
	std::bitset<8> valueHex2(valueBin2);

	// Cast the hexadecimal value into a char
	unsigned char c1 = static_cast<unsigned char>(valueHex1.to_ulong());
	unsigned char c2 = static_cast<unsigned char>(valueHex2.to_ulong());

	if (sendCommand) {
		// DEBUG
		std::cout << "Command : Going BACKWARD" << std::endl;

		// Send the message
		DWORD dwWriteBytes;
		WriteFile(g_hPort, &c1, 1, &dwWriteBytes, NULL);
		Sleep(10);
		WriteFile(g_hPort, &c2, 1, &dwWriteBytes, NULL);
		std::cout << "Msg send : BACKWARD" << std::endl;
	}
}

void Robot::sendRight(bool debug)
{
	int gainMotorR, gainMotorL;

	if (!debug) {
		gainMotorR = this->gainMotorRRight;
		gainMotorL = this->gainMotorLRight;
	}
	else {
		gainMotorR = this->gainMotor1;
		gainMotorL = this->gainMotor2;
	}

	int valueR = 32 + gainMotorR;
	int valueL = 31 - gainMotorL;

	// Conversion from decimal to binary
	std::string valueBin1 = convertFromDecTo6BitsBinary(valueR);
	std::string valueBin2 = convertFromDecTo6BitsBinary(valueL);

	// Add the "command" part
	valueBin1.append("01");
	valueBin2.append("10");

	// Reverse the binary value
	std::reverse(valueBin1.begin(), valueBin1.end());
	std::reverse(valueBin2.begin(), valueBin2.end());

	// Convert to the hexadecimal value
	std::bitset<8> valueHex1(valueBin1);
	std::bitset<8> valueHex2(valueBin2);

	// Cast the hexadecimal value into a char
	unsigned char c1 = static_cast<unsigned char>(valueHex1.to_ulong());
	unsigned char c2 = static_cast<unsigned char>(valueHex2.to_ulong());

	if (sendCommand) {
		// DEBUG
		std::cout << "Command : Going RIGHT" << std::endl;

		// Send the message
		DWORD dwWriteBytes;
		WriteFile(g_hPort, &c1, 1, &dwWriteBytes, NULL);
		Sleep(10);
		WriteFile(g_hPort, &c2, 1, &dwWriteBytes, NULL);
		std::cout << "Msg send : RIGHT" << std::endl;
	}
}

void Robot::sendLeft(bool debug)
{
	int gainMotorR, gainMotorL;

	if (!debug) {
		gainMotorR = this->gainMotorRLeft;
		gainMotorL = this->gainMotorLLeft;
	}
	else {
		gainMotorR = this->gainMotor1;
		gainMotorL = this->gainMotor2;
	}

	int valueR = 31 - gainMotorR;
	int valueL = 32 + gainMotorL;

	// Conversion from decimal to binary
	std::string valueBin1 = convertFromDecTo6BitsBinary(valueR);
	std::string valueBin2 = convertFromDecTo6BitsBinary(valueL);

	// Add the "command" part
	valueBin1.append("01");
	valueBin2.append("10");

	// Reverse the binary value
	std::reverse(valueBin1.begin(), valueBin1.end());
	std::reverse(valueBin2.begin(), valueBin2.end());

	// Convert to the hexadecimal value
	std::bitset<8> valueHex1(valueBin1);
	std::bitset<8> valueHex2(valueBin2);

	// Cast the hexadecimal value into a char
	unsigned char c1 = static_cast<unsigned char>(valueHex1.to_ulong());
	unsigned char c2 = static_cast<unsigned char>(valueHex2.to_ulong());

	if (sendCommand) {
		// DEBUG
		std::cout << "Command : Going LEFT" << std::endl;

		// Send the message
		DWORD dwWriteBytes;
		WriteFile(g_hPort, &c1, 1, &dwWriteBytes, NULL);
		Sleep(10);
		WriteFile(g_hPort, &c2, 1, &dwWriteBytes, NULL);
		std::cout << "Msg send : LEFT" << std::endl;
	}
}

void Robot::sendForward(bool debug)
{
	int gainMotorR, gainMotorL;

	if (!debug) {
		gainMotorR = this->gainMotorRForward;
		gainMotorL = this->gainMotorLForward;
	}
	else {
		gainMotorR = this->gainMotor1;
		gainMotorL = this->gainMotor2;
	}

	int valueR = 32 + gainMotorR;
	int valueL = 32 + gainMotorL;

	// Conversion from decimal to binary
	std::string valueBin1 = convertFromDecTo6BitsBinary(valueR);
	std::string valueBin2 = convertFromDecTo6BitsBinary(valueL);

	// Add the "command" part
	valueBin1.append("01");
	valueBin2.append("10");

	// Reverse the binary value
	std::reverse(valueBin1.begin(), valueBin1.end());
	std::reverse(valueBin2.begin(), valueBin2.end());

	// Convert to the hexadecimal value
	std::bitset<8> valueHex1(valueBin1);
	std::bitset<8> valueHex2(valueBin2);

	// Cast the hexadecimal value into a char
	unsigned char c1 = static_cast<unsigned char>(valueHex1.to_ulong());
	unsigned char c2 = static_cast<unsigned char>(valueHex2.to_ulong());

	if (sendCommand) {
		// DEBUG
		std::cout << "Command : Going FORWARD" << std::endl;

		// Send the message
		DWORD dwWriteBytes;
		WriteFile(g_hPort, &c1, 1, &dwWriteBytes, NULL);
		Sleep(10);
		WriteFile(g_hPort, &c2, 1, &dwWriteBytes, NULL);
		std::cout << "Msg send : FORWARD" << std::endl;
	}
}




