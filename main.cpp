/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
*/

#include "camera.h"

int main(int argc, char** argv) {

	Camera cam(0);
	Mat image, subImage, circlesImage;
	Ptr<BackgroundSubtractor> pKNN = createBackgroundSubtractorKNN();

	if (cam.cameraCalib(false) != 0) {
		cout << "Calibration error" << endl;
		return(-1);
	}

	if (cam.cameraCorr() != 0) {
		cout << "Rectification error" << endl;
		return(-2);
	}

	cam.getCap().set(CAP_PROP_AUTOFOCUS, 0);

	while (waitKey(10) != 27) {
		cam.getCap().read(image);
		remap(image, image, cam.getMap1(), cam.getMap2(), INTER_NEAREST);
		imshow("Image", image);
		subImage = cam.subtractionBack(image, pKNN);
		imshow("Background Subtraction", subImage);
		circlesImage = cam.circlesDetection(image, subImage);
		imshow("Circles Detection", circlesImage);
	}

}
