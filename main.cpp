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
	clock_t dT = clock();
	clock_t t = clock();

	// Initialization of the camera and the robot
	Camera cam(0);
	cv::Mat image;
	Robot robot = Robot();
	AreaClassification classif = AreaClassification();

	// Blue LED color
	cv::Scalar IF_lower(100, 34, 60), IF_upper(175, 128, 244);

	// Background subtractor initialization
	cv::Ptr<cv::BackgroundSubtractor> pKNN = cv::createBackgroundSubtractorKNN();

	// Calibration of the camera
	if (cam.cameraCalib(false) != 0) {
		std::cout << "Calibration error" << endl;
		return(-1);
	}
	cam.cameraCorr(1.73);

	// Last Known position of the LEDs
	infraredLight lastKnownTOP = infraredLight(), lastKnownBOTTOM = infraredLight();
	bool found = false;
	int notFoundCount = 0;
	bool error = false;
	cv::Point top, bottom;

	// Kalman filter
	int stateSize = 4, measSize = 2, contrSize = 0;
	unsigned int type = CV_32F;
	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
	cv::Mat state(stateSize, 1, type);  // (x, y, v_x, v_y) : state vector
	cv::Mat meas(measSize, 1, type); // (z_x, z_y) : measurements vector
	cv::setIdentity(kf.transitionMatrix); // Transition matrix : describes relationship between model parameters at step k and at step k + 1
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0, 0) = float(1.0);
	kf.measurementMatrix.at<float>(1, 1) = float(1.0);
	kf.processNoiseCov.at<float>(0, 0) = float(0.01);
	kf.processNoiseCov.at<float>(1, 1) = float(0.01);
	kf.processNoiseCov.at<float>(2, 2) = float(5.0);
	kf.processNoiseCov.at<float>(3, 3) = float(5.0);
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(0.1));

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
		//imshow("Image before", image);
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
		//classif.displayIdentification(image);

		// Update robot data
		if (classif.getLastKnownBOTTOM().getCoord() != cv::Point(-1, -1) && classif.getLastKnownTOP().getCoord() != cv::Point(-1, -1)) {
			notFoundCount = 0;
			found = true;
			robot.updatePosition(classif.getLastKnownBOTTOM().getCoord(), classif.getLastKnownTOP().getCoord());
			//robot.displayImagePosition(image);
		}
		else {
			notFoundCount++;
			if (notFoundCount >= 50) {
				found = false;
			}
		}

		// Update Kalman filter
		if (found) {
			meas.at<float>(0, 0) = float(robot.getImagePosition().x);
			meas.at<float>(1, 0) = float(robot.getImagePosition().y);
			// First detection
			if (classif.getPreviousInfraredVector().empty())
			{
				kf.errorCovPre.at<float>(0, 0) = 1.0;
				kf.errorCovPre.at<float>(1, 1) = 1.0;
				kf.errorCovPre.at<float>(2, 2) = 1.0;
				kf.errorCovPre.at<float>(3, 3) = 1.0;

				state.at<float>(0, 0) = meas.at<float>(0, 0);
				state.at<float>(1, 0) = meas.at<float>(1, 0);
				state.at<float>(2, 0) = 0;
				state.at<float>(3, 0) = 0;

				kf.statePost = state;
			}
			else {
				// Correction Kalman filter
				kf.correct(meas);
				// Update the matrix F
				kf.transitionMatrix.at<float>(0, 2) = float(dT) / 1000;
				kf.transitionMatrix.at<float>(1, 3) = float(dT) / 1000;
				// Predict the next position
				state = kf.predict();
				cv::circle(image, cv::Point(int(state.at<float>(0, 0)), int(state.at<float>(1, 0))), 5, cv::Scalar(255, 0, 0), -1);
			}
		}

		classif.setPreviousInfraredVector(classif.getFinalInfraredVector());
		classif.clearFinalInfraredVector();

		imshow("Image", image);
		//writer3.write(image);

		dT = (clock() - t);
		std::cout << dT << endl;
	}
	cv::destroyAllWindows();
}
