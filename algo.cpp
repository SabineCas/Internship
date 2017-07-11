/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Detection and command algorithm
*/

#include "algo.h"

Algo::Algo()
	: started(false)
	, finished(false)
	, close(false)
	, cam(0)
	, height(1.73)
{
	// Calibration of the camera
	if (cam.cameraCalib(false) != 0) {
		std::cout << "Calibration error" << std::endl;
	}
	cam.cameraCorr(this->height);

	// Initialization of the robot instance
	this->robot = Robot(height, cam.getIntrinsicParameters().at<double>(0, 0),
		cam.getIntrinsicParameters().at<double>(1, 1), cam.getIntrinsicParameters().at<double>(0, 2),
		cam.getIntrinsicParameters().at<double>(1, 2));
}

void Algo::run()
{
	// Time between two frame in ms
	clock_t dT = clock(), t = clock();

	// Variable that store the data from the current frame
	cv::Mat image;

	// Infrared LED color
	cv::Scalar IF_lower(100, 30, 40), IF_upper(250, 250, 250);
	AreaClassification classif = AreaClassification();

	// Kalman filter
	Kalman kalman = Kalman();
	bool found = false, error = false;
	int notFoundCount = 0;

	// Algorithm is starting
	started = true;

	// Recorder of the camera's images
	/*cv::Size size2 = cv::Size(int(cam.getCap().get(cv::CAP_PROP_FRAME_WIDTH)), int(cam.getCap().get(cv::CAP_PROP_FRAME_HEIGHT)));
	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	cv::VideoWriter writer3("../data/Result1.avi", codec, 10, size2, true);
	writer3.open("../data/Result1.avi", codec, 20, size2, true);*/

	// Windows for the keyboard event
	cv::imshow("DEBUG", cv::Mat::zeros(cv::Size(5, 5), CV_32F));

	while (!close) {
		// Update of the timer
		t = clock();

		// Get the next frame (using mutex to avoid conflict access)
		mtx.lock();
		cam.getCap().read(image);
		mtx.unlock();
		if (image.size() == cv::Size(0, 0)) {
			break;
		}

		// Fix the optical distorsion of the camera
		// remap(image, image, cam.getMap1(), cam.getMap2(), cv::INTER_NEAREST);

		// Detecttion of the LED
		cv::cvtColor(image, image, CV_BGR2HSV);
		classif.setInfraredVector(cam.ledDetection(image, IF_lower, IF_upper));
		cv::cvtColor(image, image, CV_HSV2BGR);

		// Classification of the different areas
		classif.updateCurrentFromPrevious();
		classif.updatePreviousFromCurrent();
		classif.updateIdentification(dT);
		classif.identifyLastKnownLocation();
		if (this->displayIdentification) {
			classif.displayIdentification(image);
		}

		// Display the position on the picture that we want the robot reaches
		robot.displayDesiredPosition(image);

		// Update robot position and orientation
		if (classif.getLastKnownBOTTOM().getCoord() != cv::Point(-1, -1) && classif.getLastKnownTOP().getCoord() != cv::Point(-1, -1)) {
			notFoundCount = 0;
			found = true;
			robot.updatePosition(classif.getLastKnownTOP().getCoord(), classif.getLastKnownBOTTOM().getCoord());
			if (this->displayPosition) {
				robot.displayImagePosition(image);
			}
			if (this->displayOrientation) {
				robot.displayImageOrientation(image, classif.getLastKnownTOP().getCoord());
			}
		}
		else {
			notFoundCount++;
			if (notFoundCount >= 25) {
				found = false;
			}
		}

		// Update Kalman filter
		if (found) {
			kalman.updateMeas(int(robot.getImagePosition().x), int(robot.getImagePosition().y));
			if (classif.getPreviousInfraredVector().empty()) {
				kalman.initiateKalmanFilter();
			}
			else {
				kalman.predictKalmanFilter(dT);
			}
			if (this->displayKalman) {
				kalman.displayEstimatePosition(image);
			}
		}

		// Update the previous LEDs vector
		classif.setPreviousInfraredVector(classif.getFinalInfraredVector());
		classif.clearFinalInfraredVector();

		// Security : if we don't see the robot on the picture for too long, we send stop to the robot
		if (found) {
			if (this->debug) {
				this->robot.sendCommandToRobotDEBUG();
			}
			else {
				this->robot.sendCommandToRobotArranged(dT);
			}
		}
		else {
			this->robot.sendStop();
		}

		// Record the video flux
		// writer3.write(image);

		// Update the image of the interface with a resizement fo the image, so it will fit
		cv::resize(image, image, cv::Size(640, 480));
		cv::cvtColor(image, image, CV_BGR2RGB);
		QImage img(image.data, image.cols, image.rows, QImage::Format_RGB888);
		interf->SetImage(img);

		// Update the timer between two frame
		dT = (clock() - t);
	}

	// Stop the robot and close the communication port
	this->robot.closeCom();

	// join the thread and close the program
	std::this_thread::sleep_for(std::chrono::seconds(1));
	interf->end();
	finished = false;
}

void Algo::start()
{
	runThr = std::thread(&Algo::run, this);
}

void Algo::forceQuit()
{
	if (!this->started)
		return;
	if (!this->close)
		this->close = true;
	if (!this->finished)
		this->runThr.join();
}

void Algo::setInterface(MainInterface *i)
{
	this->interf = i;
}

void Algo::setDistanceAreaLight(int i)
{
	distanceAreaLight = i;
}

void Algo::setFreqLED1(int i)
{
	timeLED1 = i;
}

void Algo::setFreqLED2(int i)
{
	timeLED2 = i;
}

void Algo::setDisplayPosition(bool val)
{
	this->displayPosition = val;
}

void Algo::setDisplayOrientation(bool val)
{
	this->displayOrientation = val;
}

void Algo::setDisplayIdentification(bool val)
{
	this->displayIdentification = val;
}

void Algo::setDisplayKalman(bool val)
{
	this->displayKalman = val;
}

void Algo::setDesiredPoint(int x, int y)
{
	x = double(x) * this->cam.getCap().get(CV_CAP_PROP_FRAME_WIDTH) / 640;
	y = double(y) * this->cam.getCap().get(CV_CAP_PROP_FRAME_HEIGHT) / 480;
	this->robot.setDesiredPosition(cv::Point(x, y));
}

void Algo::setNbRobot(int r)
{
	this->nbRobot = r;
}

void Algo::setHeight(int h)
{
	// Conversion in meters
	this->height = double(h) / 100;
	// Update the value
	this->robot.setHeight(this->height);
}

void Algo::sendCommand(bool c)
{
	this->robot.setSendCommand(c);
}

void Algo::setGainMotor1(int g)
{
	this->robot.setGainMotor1(g);
}

void Algo::setGainMotor2(int g)
{
	this->robot.setGainMotor2(g);
}

void Algo::setResolution(int width, int height)
{
	// Ask the mutex
	mtx.lock();

	// Change the resolution
	cam.getCap().set(CV_CAP_PROP_FRAME_WIDTH, width);
	cam.getCap().set(CV_CAP_PROP_FRAME_HEIGHT, height);
	std::cout << "Setting : " << cam.getCap().get(CV_CAP_PROP_FRAME_WIDTH) << " x " <<
		cam.getCap().get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;

	// Release the mutex
	mtx.unlock();
}

void Algo::setDebug(bool d)
{
	this->debug = d;
}

void Algo::loadGainFile(bool d)
{
	if (this->debug && d) {
		this->robot.loadGainFile();
	}
}
