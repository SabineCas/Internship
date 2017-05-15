/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
*/

#include "camera.h"

int main(int argc, char** argv) {

	Camera cam(0);
	Mat image, subImage, previousSubImage, circlesImage;
	Ptr<BackgroundSubtractor> pKNN = createBackgroundSubtractorKNN();

	if (cam.cameraCalib(false) != 0) {
		cout << "Calibration error" << endl;
		return(-1);
	}

	if (cam.cameraCorr() != 0) {
		cout << "Rectification error" << endl;
		return(-2);
	}

	cam.getCap().set(cv::CAP_PROP_FOCUS, false);
	cam.getCap().set(CAP_PROP_AUTOFOCUS, false);

	while (waitKey(10) != 27) {
		
		cam.getCap().read(image);
		remap(image, image, cam.getMap1(), cam.getMap2(), INTER_NEAREST);
		//imshow("Image", image);

		/* Strategy 1 : Background subtraction and Hough transformation */
		subImage.copyTo(previousSubImage);
		subImage = cam.subtractionBack(2, image, pKNN, previousSubImage);
		imshow("Background Subtraction", subImage);
		cam.circlesDetection(image, subImage);
		circlesImage = cam.displayCircles(image);
		rectangle(image, cam.getBoundingBoxObs(), Scalar(255,0,0), 2, 8, 0);
		imshow("Circles Detection", circlesImage);

		/* Strategy 2 : Object tracking and learning --> OpenTDL */

	}

}
