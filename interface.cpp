#include "interface.h"

WindowInterface::WindowInterface(int nbRobot, int valueDist)
{
	this->stateButtonPosition = false;
	this->stateButtonOrientation = false;
	this->stateButtonIdentification = false;
	this->nbRobot = nbRobot;
	this->valueDistanceArea = valueDist;
	this->factorResolution = 3;
	this->previousFactorResolution = 3;
}

int WindowInterface::show()
{
	// Time between two frame in ms
	clock_t dT = clock(), t = clock();

	// Pressed key
	int key = cv::waitKey(10);

	// Initialization of the camera and the robot
	Camera cam(0);
	cv::Mat image;

	// Calibration of the camera
	if (cam.cameraCalib(false) != 0) {
		std::cout << "Calibration error" << std::endl;
	}
	const static double height = 1.73;
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

	// Interface
	cv::namedWindow(titleWindows, CV_WINDOW_KEEPRATIO);
	cv::createTrackbar("Resolution", this->titleWindows, &(this->factorResolution), 3, NULL);
	cv::createButton("Display the position of the robot", callbackButtonPosition, &(this->stateButtonPosition), CV_CHECKBOX, true);
	cv::createButton("Display the orientation of the robot", callbackButtonOrientation, &(this->stateButtonOrientation), CV_CHECKBOX, false);
	cv::createButton("Display the identification of each area", callbackButtonIdentification, &(this->stateButtonIdentification), CV_CHECKBOX, false);

	while (key != 27) {
		if (this->factorResolution != this->previousFactorResolution) {
			this->previousFactorResolution = this->factorResolution;
			// Set the new resolution of the frame
			cam.getCap().set(CV_CAP_PROP_FRAME_WIDTH, 640 * (1.0 + double(this->factorResolution)));
			cam.getCap().set(CV_CAP_PROP_FRAME_HEIGHT, 480 * (1.0 + double(this->factorResolution)));
			std::cout << cam.getCap().get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cam.getCap().get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
		}
		
		// Update of the timer
		t = clock();

		// Get the next frame
		cam.getCap().read(image);
		if (image.size() == cv::Size(0, 0)) {
			break;
		}

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
		if (this->stateButtonIdentification) {
			classif.displayIdentification(image);
		}

		// Update robot position and orientation
		if (classif.getLastKnownBOTTOM().getCoord() != cv::Point(-1, -1) && classif.getLastKnownTOP().getCoord() != cv::Point(-1, -1)) {
			notFoundCount = 0;
			found = true;
			robot.updatePosition(classif.getLastKnownTOP().getCoord(), classif.getLastKnownBOTTOM().getCoord());
			if (this->stateButtonPosition) {
				robot.displayImagePosition(image);
			}
			if (this->stateButtonOrientation) {
				robot.displayImageOrientation(image, classif.getLastKnownTOP().getCoord());
			}
			
		}
		else {
			notFoundCount++;
			if (notFoundCount >= 50) {
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
			//kalman.displayEstimatePosition(image);
		}

		// Update the previous LEDs vector
		classif.setPreviousInfraredVector(classif.getFinalInfraredVector());
		classif.clearFinalInfraredVector();

		//cv::resize(image, image, cv::Size(640, 480));
		IplImage img1 = (IplImage)image;
		cvShowImage(titleWindows, &img1);
		//writer3.write(image);

		dT = (clock() - t);
		std::cout << dT << std::endl;
		key = cv::waitKey(10);

		// Control keys for the robot
		if (key == 37) {
			std::cout << "LEFT" << std::endl;
		}
		else if (key == 38) {
			std::cout << "UP" << std::endl;
		}
		else if (key == 39) {
			std::cout << "RIGHT" << std::endl;
		}
		else if (key == 40) {
			std::cout << "DOWN" << std::endl;
		}
	}
	cv::destroyAllWindows();
	return(0);
}

void WindowInterface::callbackButtonPosition(int state, void * data)
{
	int * temp = (int *) data;
	*temp = state;
	std::cout << "Position : " << (int)*temp << std::endl;
}

void WindowInterface::callbackButtonOrientation(int state, void * data)
{
	int * temp = (int *)data;
	*temp = state;
	std::cout << "Orientation : " << state << std::endl;
}

void WindowInterface::callbackButtonIdentification(int state, void * data)
{
	int * temp = (int *)data;
	*temp = state;
	std::cout << "Identification : " << state << std::endl;
}

