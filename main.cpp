/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Main function
*/

#include "robot.h"
#include <windows.h>

int main(int argc, char** argv) {
	// Time between two frame in ms
	double dT = 0;

	// Initialization of the camera and the robot
	Camera cam(0);
	Robot robot = Robot();

	// Blue LED color
	cv::Scalar blue_lower(0, 0, 233), blue_upper(165, 255, 255), IF_lower(143, 34, 60), IF_upper(175, 128, 244);

	// Last Known position of the LEDs
	LightArea lastKnownTOP = LightArea(), lastKnownBOTTOM = LightArea();

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
	/*cam.getCap().set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cam.getCap().set(cv::CAP_PROP_FRAME_HEIGHT, 720);*/
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
		imshow("Image before", image);

		// Detecttion of the LED
		cv::cvtColor(image, image, CV_BGR2HSV);
		blueVector = cam.ledDetection(image, IF_lower, IF_upper);
		cv::cvtColor(image, image, CV_HSV2BGR);

		updatePreviousToCurrent(previousBlueVector, blueVector, finalBlueVector);
		updateCurrentToPrevious(previousBlueVector, blueVector, finalBlueVector);
		updateIdentification(previousBlueVector, finalBlueVector, dT);

		for (std::vector<LightArea>::size_type i = 0; i < finalBlueVector.size(); i++) {
			if (finalBlueVector[i].getIdentification() != "UNKNOWN") { // if top or bottom display
				cv::putText(image, finalBlueVector[i].getIdentification(), finalBlueVector[i].getCoord(), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, 8, false);
				int indice = finalBlueVector[i].findIn(previousBlueVector);
				if (indice != -1) {
					if (finalBlueVector[i].getIdentification() == "TOP" && previousBlueVector[finalBlueVector[i].findIn(previousBlueVector)].getIdentification() == "TOP") { // et qu'il est aussi dans previous --> TO DO
						lastKnownTOP = finalBlueVector[i];
					}
					else if (finalBlueVector[i].getIdentification() == "BOTTOM" && previousBlueVector[finalBlueVector[i].findIn(previousBlueVector)].getIdentification() == "BOTTOM") {
						lastKnownBOTTOM = finalBlueVector[i];
					}
				}
			}
		}

		if (!(finalBlueVector.empty())) {
			cv::Point top(0, 0), bottom(0, 0);
			bool lastTOP = lastKnownTOP.isContainedIn(previousBlueVector), lastBOTTOM = lastKnownBOTTOM.isContainedIn(previousBlueVector);
			bool currentTOP = lastKnownTOP.isContainedIn(finalBlueVector), currentBOTTOM = lastKnownBOTTOM.isContainedIn(finalBlueVector);
			bool error = false;
			if (currentTOP || lastTOP) {
				top = lastKnownTOP.getCoord();
			}
			else {
				error = true;
			}
			if (currentBOTTOM || lastBOTTOM) {
				bottom = lastKnownBOTTOM.getCoord();
			}
			else {
				error = true;
			}
			if (!error) {
				// Update the position of the robot and the searching bounding box area
				robot.setImagePosition((top.x + bottom.x) / 2, (top.y + bottom.y) / 2);
				// Display the estimated position
				circle(image, robot.getImagePosition(), 5, cv::Scalar(0, 0, 255), -1, 8, 0);
			}
		}

		previousBlueVector = copyLAvector(finalBlueVector, previousBlueVector);
		finalBlueVector.clear();



		//writer2.write(image);
		imshow("Image", image);

		dT = (clock() - t);
		//std::cout << dT << endl;
	}
	cv::destroyAllWindows();
}
