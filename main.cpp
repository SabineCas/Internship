/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Main function
*/

#include "robot.h"
#include <windows.h>

bool operator<(cv::Point const& a, cv::Point const& b)
{
	return (a.x < b.x) || (a.x == b.x && a.y < b.y);
}

int main(int argc, char** argv) {
	// Time between two frame in ms
	const static int time_frame = 50;

	// Initialization of the camera and the robot
	Camera cam(0);
	Robot robot = Robot();

	// Blue LED color
	cv::Scalar blue_lower(0, 0, 233);
	cv::Scalar blue_upper(165, 255, 255);
	cv::Scalar robot_lower(0, 0, 129);
	cv::Scalar robot_upper(179, 86, 255);


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
	//cam.getCap().set(cv::CAP_PROP_FPS, 20);
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
		//remap(image, image, cam.getMap1(), cam.getMap2(), cv::INTER_NEAREST);
		//imshow("Image", image);

		// Detecttion of the LED
		cv::cvtColor(image, image, CV_BGR2HSV);
		blueVector = cam.ledDetection(image, blue_lower, blue_upper);
		cv::cvtColor(image, image, CV_HSV2BGR);

		bool found = false;

		// For each detected area in the previous frame, we add to the final vector every area that we can also detected in this frame
		// and update their value (visible, coordinate, ...) 
		for (std::vector<LightArea>::size_type i = 0; i < previousBlueVector.size(); i++) {
			found = false;
			for (std::vector<LightArea>::size_type j = 0; j < blueVector.size(); j++) {
				if (previousBlueVector[i].updateCoord(blueVector[j].getCoord())) {
					blueVector[j].setNumArea(previousBlueVector[i].getNumArea());
					blueVector[j].setVisible(true);
					blueVector[j].setLEDTimeON(previousBlueVector[i].getLEDTimeON());
					blueVector[j].setLEDTimeOFF(previousBlueVector[i].getLEDTimeOFF());
					if (!blueVector[j].isContainedIn(finalBlueVector)) {
						finalBlueVector.push_back(blueVector[j]);
					}
					found = true;
					break;
				}
			}
			if (!found) {
				finalBlueVector.push_back(LightArea(false, previousBlueVector[i].getCoord(), previousBlueVector[i].getNumArea(), previousBlueVector[i].getLEDTimeON(), previousBlueVector[i].getLEDTimeOFF(), previousBlueVector[i].getIdentification()));
			}
		}

		// For each detected area in this frame, we add to the final vector every new area from the previous frame
		for (std::vector<LightArea>::size_type i = 0; i < blueVector.size(); i++) {
			// The LightArea is not in the final vector, we add it, otherwise we just update it (but usually, it will not happen)
			if (!blueVector[i].isContainedIn(finalBlueVector)) {
				finalBlueVector.push_back(LightArea(true, blueVector[i].getCoord(), finalBlueVector.size(), blueVector[i].getLEDTimeON(), blueVector[i].getLEDTimeON(), blueVector[i].getIdentification()));
			}
		}

		// Now that the final vector have the new detected and the previous updated areas, we can study the blinking frequency of every area
		// to determine if that matches with the LED frequency
		int indice;
		for (std::vector<LightArea>::size_type i = 0; i < finalBlueVector.size(); i++) {
			indice = finalBlueVector[i].findIn(previousBlueVector);
			if (indice == -1) {
				finalBlueVector[i].areaBlinkFreq((clock() - t), false);
			} else {
				finalBlueVector[i].areaBlinkFreq((clock() - t), previousBlueVector[indice].getVisible());
			}
			if (finalBlueVector[i].getVisible()) { // if top or bottom display
				std::string test = finalBlueVector[i].getIdentification();
				cv::putText(image, test, finalBlueVector[i].getCoord(), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, 8, false);
			}
		}

		previousBlueVector = copyLAvector(finalBlueVector, previousBlueVector);
		finalBlueVector.clear();

		//if (!(blueVector.empty())) {
		//	// Update the position of the robot and the searching bounding box area
		//	robot.setImagePosition(blueVector[0].x, blueVector[0].y);
		//	// Display the estimated position
		//	circle(image, robot.getImagePosition(), 5, cv::Scalar(0, 0, 255), -1, 8, 0);
		//}

		//writer2.write(image);
		imshow("Image", image);

		std::cout << (clock() - t) << endl;
		
		//// Setting the frequency of the loop
		//if (time_frame - (clock() - t) > 0) {
		//	//cout << "Loop : " << time_frame - (clock() - t) << endl;
		//	Sleep(time_frame - (clock() - t));
		//}
		//else {
		//	//cout << time_frame - (clock() - t) << endl;
		//}
	}
	cv::destroyAllWindows();
}
