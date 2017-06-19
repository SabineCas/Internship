#include "algo.h"
#include "infraredLight.h"
#include <iostream>


Algo::Algo()
	: started(false)
	, finished(false)
	, close(false)
	, cam(0)
{}

void Algo::run()
{
	// Time between two frame in ms
	clock_t dT = clock(), t = clock();

	// Initialization of the camera and the robot
	//Camera cam(0);
	cv::Mat image;

	// Calibration of the camera
	if (cam.cameraCalib(false) != 0) {
		std::cout << "Calibration error" << endl;
	}
	double height = 1.73;
	cam.cameraCorr(height);

	// Infrared LED color
	cv::Scalar IF_lower(100, 34, 60), IF_upper(175, 128, 244);
	AreaClassification classif = AreaClassification();

	// Robot
	Robot robot = Robot(height, cam.getIntrinsicParameters().at<double>(0, 0), cam.getIntrinsicParameters().at<double>(1, 1),
		cam.getIntrinsicParameters().at<double>(0, 2), cam.getIntrinsicParameters().at<double>(1, 2));

	// Kalman filter
	Kalman kalman = Kalman();
	bool found = false, error = false;
	int notFoundCount = 0;

	started = true;
	

	while (!close) {
		// Update of the timer
		t = clock();

		// Get the next frame
		mtx.lock();
		cam.getCap().read(image);
		mtx.unlock();
		if (image.size() == cv::Size(0, 0)) {
			break;
		}

		dT = (clock() - t);
		std::cout << dT << " ";

		// Fix the optical distorsion of the camera
		remap(image, image, cam.getMap1(), cam.getMap2(), cv::INTER_NEAREST);
		//writer2.write(image);

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
			if (notFoundCount >= 50) {
				found = false;
			}
		}

		dT = (clock() - t);
		std::cout << dT << " ";

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

		dT = (clock() - t);
		std::cout << dT << " ";

		// Update the previous LEDs vector
		classif.setPreviousInfraredVector(classif.getFinalInfraredVector());
		classif.clearFinalInfraredVector();

		//writer3.write(image);
		dT = (clock() - t);
		std::cout << dT << " ";

		cv::cvtColor(image, image, CV_BGR2RGB);
		QImage img(image.data, image.cols, image.rows, QImage::Format_RGB888);
		interf->SetImage(img);

		dT = (clock() - t);
		std::cout << dT << std::endl;

		//std::cout << "value : " << value << std::endl;
		
	}

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

void Algo::setResolution(int width, int height)
{
	mtx.lock();
	cam.getCap().set(CV_CAP_PROP_FRAME_WIDTH, width);
	cam.getCap().set(CV_CAP_PROP_FRAME_HEIGHT, height);
	std::cout << "Setting : " << cam.getCap().get(CV_CAP_PROP_FRAME_WIDTH) << " x " <<
		cam.getCap().get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
	mtx.unlock();
}

