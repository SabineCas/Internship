/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Main function
*/

#include "robot.h"
#include "areaClassification.h"
#include <windows.h>

int main(int argc, char** argv) {
	// Time between two frame in ms
	clock_t dT = clock(), t = clock();

	// Initialization of the camera and the robot
	Camera cam(0);
	cv::Mat image;
	
	// Calibration of the camera
	if (cam.cameraCalib(false) != 0) {
		std::cout << "Calibration error" << endl;
		return(-1);
	}
	const static double height = 1.73;
	cam.cameraCorr(height);

	// Infrared LED color
	cv::Scalar IF_lower(100, 34, 60), IF_upper(175, 128, 244);
	AreaClassification classif = AreaClassification();

	// Background subtractor initialization
	cv::Ptr<cv::BackgroundSubtractor> pKNN = cv::createBackgroundSubtractorKNN();

	// Robot
	Robot robot = Robot(height, cam.getIntrinsicParameters().at<double>(0, 0), cam.getIntrinsicParameters().at<double>(1, 1),
		cam.getIntrinsicParameters().at<double>(0, 2), cam.getIntrinsicParameters().at<double>(1, 2));

	// Kalman filter
	Kalman kalman = Kalman();
	bool found = false, error = false;
	int notFoundCount = 0;

	// DEBUG
	cv::Size size2 = cv::Size(int(cam.getCap().get(cv::CAP_PROP_FRAME_WIDTH)), int(cam.getCap().get(cv::CAP_PROP_FRAME_HEIGHT)));
	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	cv::VideoWriter writer2("../data/Result1.avi", codec, 10, size2, true);
	writer2.open("../data/Result1.avi", codec, 20, size2, true);
	cv::VideoWriter writer3("../data/Result2.avi", codec, 10, size2, true);
	writer3.open("../data/Result2.avi", codec, 20, size2, true);

	while (cv::waitKey(10) != 27) {
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
		classif.displayIdentification(image);

		// Update robot position and orientation
		if (classif.getLastKnownBOTTOM().getCoord() != cv::Point(-1, -1) && classif.getLastKnownTOP().getCoord() != cv::Point(-1, -1)) {
			notFoundCount = 0;
			found = true;
			robot.updatePosition(classif.getLastKnownTOP().getCoord(), classif.getLastKnownBOTTOM().getCoord());
			robot.displayImagePosition(image);
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

		imshow("Image", image);
		//writer3.write(image);

		dT = (clock() - t);
		std::cout << dT << endl;
	}
	cv::destroyAllWindows();
}
