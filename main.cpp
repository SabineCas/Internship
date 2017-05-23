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
	const static int time_frame = 50;

	// Initialization of the camera and the robot
	Camera cam(0);
	Robot robot = Robot();

	// Blue LED color
	cv::Scalar blue_lower(50, 90, 155);
	cv::Scalar blue_upper(170, 255, 255);
	cv::Scalar robot_lower(0, 0, 129);
	cv::Scalar robot_upper(179, 86, 255);


	cv::Mat image, subImage, thresImage, ANDImage, ORImage;
	std::vector<cv::Point> blueVector;
	cv::Ptr<cv::BackgroundSubtractor> pKNN = cv::createBackgroundSubtractorKNN();

	if (cam.cameraCalib(false) != 0) {
		cout << "Calibration error" << endl;
		return(-1);
	}

	if (cam.cameraCorr() != 0) {
		cout << "Rectification error" << endl;
		return(-2);
	}

	// Deactivate the autofocus of the camera
	cam.getCap().set(cv::CAP_PROP_FOCUS, false);
	cam.getCap().set(cv::CAP_PROP_AUTOFOCUS, false);
	clock_t t = clock();

	// DEBUG
	/*Size size2 = Size(cam.getCap().get(CAP_PROP_FRAME_WIDTH), cam.getCap().get(CAP_PROP_FRAME_HEIGHT));
	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	VideoWriter writer2("../data/RGBTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);
	VideoWriter writer3("../data/GrayTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);
	writer2.open("../data/RGBTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);
	writer3.open("../data/GrayTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);*/

	while (cv::waitKey(10) != 27) {
		// Update of the timer
		t = clock();

		// Get the next frame
		cam.getCap().read(image);
		if (image.size() == cv::Size(0, 0)) {
			break;
		}

		cam.setDetectedCircle(false);

		// Fix the optical distorsion of the camera
		//remap(image, image, cam.getMap1(), cam.getMap2(), cv::INTER_NEAREST);
		//imshow("Image", image);

		// Update the dynamic background and return the result from backgroung subtraction
		subImage = cam.updateBackground(image, pKNN);
		cv::cvtColor(image, image, CV_BGR2HSV);
		blueVector = cam.ledDetection(image, blue_lower, blue_upper);
		cv::cvtColor(image, image, CV_HSV2BGR);
		if (cam.evaluateMarkersPosition(blueVector)) {
			// Estimate the real position only with the markers

			// Display the estimated position

		}
		else {
			thresImage = cam.colorDetection(image, robot_lower, robot_upper);
			cv::cvtColor(image, image, CV_HSV2BGR);
			subImage = cam.improveBackSubtr(subImage);
			//cv::cvtColor(thresImage, thresImage, CV_BGR2GRAY);
			ANDImage = thresImage & subImage;
			cam.circlesDetection(ANDImage);
			cam.displayCircles(image);
			imshow("ANDImage", ANDImage);

			if (cam.getNbDetectedLED() == 0) {
				if (cam.getDetectedCircle()) {
					// Update the position of the robot and the searching bounding box area
					robot.setImagePosition((int)(cam.getBoundingBoxObs().x + cam.getBoundingBoxObs().width / 2), (int)(cam.getBoundingBoxObs().y + cam.getBoundingBoxObs().height / 2));
					// Display the estimated position
					circle(image, robot.getImagePosition(), 15, cv::Scalar(0, 0, 255), -1, 8, 0);
				}
			}
			else {
				cv::Point temp = cam.coherenceCirclesMarkers(blueVector);
				if (cam.getDetectedCircle() && temp != cv::Point(-1, -1)) {
					// Update the position of the robot and the searching bounding box area
					robot.setImagePosition(temp.x, temp.y);
					// Display the estimated position
					circle(image, robot.getImagePosition(), 15, cv::Scalar(0, 0, 255), -1, 8, 0);
				}
			}

			if (!(cam.getDetectedCircle())) {
				ORImage = thresImage | subImage;
				cam.circlesDetection(ORImage);
				cam.displayCircles(image);
				imshow("ORImage", ORImage);

				if (cam.getNbDetectedLED() == 0) {
					if (cam.getDetectedCircle()) {
						// Update the position of the robot and the searching bounding box area
						robot.setImagePosition((int)(cam.getBoundingBoxObs().x + cam.getBoundingBoxObs().width / 2), (int)(cam.getBoundingBoxObs().y + cam.getBoundingBoxObs().height / 2));
						// Display the estimated position
						circle(image, robot.getImagePosition(), 15, cv::Scalar(0, 0, 255), -1, 8, 0);
					}
				}
				else {
					cv::Point temp = cam.coherenceCirclesMarkers(blueVector);
					if (cam.getDetectedCircle() && temp != cv::Point(-1, -1)) {
						// Update the position of the robot and the searching bounding box area
						robot.setImagePosition(temp.x, temp.y);
						// Display the estimated position
						circle(image, robot.getImagePosition(), 15, cv::Scalar(0, 0, 255), -1, 8, 0);
					}
				}
			}

			if (!(cam.getDetectedCircle()) && cam.getNbDetectedLED() > 0) {
				// Update the position of the robot and the searching bounding box area
				robot.setImagePosition(blueVector[0].x, blueVector[0].y);
				// Display the estimated position
				circle(image, robot.getImagePosition(), 15, cv::Scalar(0, 0, 255), -1, 8, 0);
			}
			else if (!(cam.getDetectedCircle()) && cam.getNbDetectedLED() == 0){
				cout << "The robot is undetected" << endl;
				cam.wideringBoundingBox(WIDE_BOUNDING_BOX_X);
			}
		}
		imshow("Image", image);
		cout << (clock() - t) << endl;
		// Setting the frequency of the loop
		if (time_frame - (clock() - t) > 0) {
			//cout << "Loop : " << time_frame - (clock() - t) << endl;
			Sleep(time_frame - (clock() - t));
		}
		else {
			//cout << time_frame - (clock() - t) << endl;
		}
	}
	cv::destroyAllWindows();
}
