/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Main function
*/

#include "robot.h"
#include <windows.h>

int main(int argc, char** argv) {

	// Interface
	/*int res, typefen = MB_OK;
	res = MessageBox(NULL, "voila le message", "voila le titre", typefen);
	printf("Code de retour : %d\n", res);*/

	// Time between two frame in ms
	clock_t dT = clock();

	// Initialization of the camera and the robot
	Camera cam(0);
	Robot robot = Robot();

	// Blue LED color
	cv::Scalar blue_lower(0, 0, 233), blue_upper(165, 255, 255), IF_lower(143, 34, 60), IF_upper(175, 128, 244);

	// Last Known position of the LEDs
	LightArea lastKnownTOP = LightArea(), lastKnownBOTTOM = LightArea();
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

	cv::Mat image, subImage, thresImage, ANDImage, ORImage;
	std::vector<LightArea> blueVector, previousBlueVector, finalBlueVector;
	cv::Ptr<cv::BackgroundSubtractor> pKNN = cv::createBackgroundSubtractorKNN();

	if (cam.cameraCalib(false) != 0) {
		std::cout << "Calibration error" << endl;
		return(-1);
	}

	if (cam.cameraCorr() != 0) {
		std::cout << "Rectification error" << endl;
		return(-2);
	}

	// Deactivate the autofocus of the camera
	cam.getCap().set(cv::CAP_PROP_FOCUS, false);
	cam.getCap().set(cv::CAP_PROP_AUTOFOCUS, false);
	clock_t t = clock();

	// DEBUG
	/*cv::Size size2 = cv::Size(cam.getCap().get(cv::CAP_PROP_FRAME_WIDTH), cam.getCap().get(cv::CAP_PROP_FRAME_HEIGHT));
	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	cv::VideoWriter writer2("../data/Result1.avi", codec, 10, size2, true);
	writer2.open("../data/Result1.avi", codec, 10, size2, true);*/

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

		// Detecttion of the LED
		cv::cvtColor(image, image, CV_BGR2HSV);
		blueVector = cam.ledDetection(image, IF_lower, IF_upper);
		cv::cvtColor(image, image, CV_HSV2BGR);

		updatePreviousToCurrent(previousBlueVector, blueVector, finalBlueVector);
		updateCurrentToPrevious(previousBlueVector, blueVector, finalBlueVector);
		updateIdentification(previousBlueVector, finalBlueVector, dT);

		// Display the position of the LEDs with their identification
		for (std::vector<LightArea>::size_type i = 0; i < finalBlueVector.size(); i++) {
			if (finalBlueVector[i].getIdentification() != "UNKNOWN") {
				if (finalBlueVector[i].findIn(previousBlueVector) != -1) {
					if (finalBlueVector[i].getIdentification() == "TOP" && previousBlueVector[finalBlueVector[i].findIn(previousBlueVector)].getIdentification() == "TOP") { // et qu'il est aussi dans previous --> TO DO
						lastKnownTOP = finalBlueVector[i];
						//cv::putText(image, "TOP", finalBlueVector[i].getCoord(), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, 8, false);
					}
					else if (finalBlueVector[i].getIdentification() == "BOTTOM" && previousBlueVector[finalBlueVector[i].findIn(previousBlueVector)].getIdentification() == "BOTTOM") {
						lastKnownBOTTOM = finalBlueVector[i];
						//cv::putText(image, "BOTTOM", finalBlueVector[i].getCoord(), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, 8, false);
					}
				}
			}
		}

		if (!(previousBlueVector.empty())) {
			bool lastTOP = lastKnownTOP.findIn(previousBlueVector) != -1 && previousBlueVector[lastKnownTOP.findIn(previousBlueVector)].getIdentification() == "TOP";
			bool lastBOTTOM = lastKnownBOTTOM.findIn(previousBlueVector) != -1 && previousBlueVector[lastKnownBOTTOM.findIn(previousBlueVector)].getIdentification() == "BOTTOM";
			bool currentTOP = lastKnownTOP.findIn(finalBlueVector) != -1 && finalBlueVector[lastKnownTOP.findIn(finalBlueVector)].getIdentification() == "TOP";
			bool currentBOTTOM = lastKnownBOTTOM.findIn(finalBlueVector) != -1 && finalBlueVector[lastKnownBOTTOM.findIn(finalBlueVector)].getIdentification() == "BOTTOM";

			error = false;
			if ((currentTOP || lastTOP) && (currentBOTTOM || lastBOTTOM)) {
				top = lastKnownTOP.getCoord();
				bottom = lastKnownBOTTOM.getCoord();
			}
			else {
				error = true;
			}

			if (!error) {
				found = true;
				// Update the position of the robot and the searching bounding box area
				robot.setImagePosition((top.x + bottom.x) / 2, (top.y + bottom.y) / 2);
				// Display the estimated position
				circle(image, robot.getImagePosition(), 5, cv::Scalar(0, 0, 255), -1, 8, 0);
			}
			else {
				notFoundCount++;
				if (notFoundCount >= 100) {
					found = false;
				}
			}
		}

		// Update Kalman filter
		if (found) {
			notFoundCount = 0;
			meas.at<float>(0, 0) = float(robot.getImagePosition().x);
			meas.at<float>(1, 0) = float(robot.getImagePosition().y);

			// First detection
			if (previousBlueVector.empty())
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

				found = true;
			}
			else {
				// Correction Kalman filter
				kf.correct(meas);
			}
		}

		previousBlueVector = copyLAvector(finalBlueVector, previousBlueVector);
		finalBlueVector.clear();

		// Kalman filter
		if (error) {
			// Update the matrix F
			kf.transitionMatrix.at<float>(0, 2) = float(dT) / 1000;
			kf.transitionMatrix.at<float>(1, 3) = float(dT) / 1000;

			// Predict the next position
			state = kf.predict();
			cv::circle(image, cv::Point(int(state.at<float>(0, 0)), int(state.at<float>(1, 0))), 5, cv::Scalar(255, 0, 0), -1);
		}

		//writer2.write(image);
		imshow("Image", image);

		dT = (clock() - t);
		//std::cout << dT << endl;
	}
	cv::destroyAllWindows();
}
