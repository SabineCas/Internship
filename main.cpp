/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
*/

#include "camera.h"

int main(int argc, char** argv) {

	Camera cam(0);
	Mat image, subImage, previousSubImage, circlesImage, thresImage;
	Ptr<BackgroundSubtractor> pKNN = createBackgroundSubtractorKNN();

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
	cam.getCap().set(CAP_PROP_AUTOFOCUS, false);

	cout << "FRAME : " << cam.getCap().get(CV_CAP_PROP_FPS) << endl;

	// DEBUG
	/*Size size2 = Size(cam.getCap().get(CAP_PROP_FRAME_WIDTH), cam.getCap().get(CAP_PROP_FRAME_HEIGHT));
	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	VideoWriter writer2("../data/RGBTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);
	VideoWriter writer3("../data/GrayTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);
	writer2.open("../data/RGBTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);
	writer3.open("../data/GrayTest1.avi", codec, cam.getCap().get(CV_CAP_PROP_FPS), size2, true);*/

	while (waitKey(10) != 27 || image.empty()) {
		
		cam.getCap().read(image);
		remap(image, image, cam.getMap1(), cam.getMap2(), INTER_NEAREST);
		//imshow("Image", image);

		/* Strategy 1 : Background subtraction and Hough transformation */
		/*subImage.copyTo(previousSubImage);
		subImage = cam.subtractionBack(2, image, pKNN, previousSubImage);
		imshow("Background Subtraction", subImage);
		cam.circlesDetection(image, subImage);
		circlesImage = cam.displayCircles(image);
		rectangle(image, cam.getBoundingBoxObs(), Scalar(255, 0, 0), 2, 8, 0);
		imshow("Circles Detection", circlesImage);*/

		/* Strategy 2 : Background subtraction, Hough transformation and color recognition */
		/*subImage.copyTo(previousSubImage);
		subImage = cam.subtractionBack(2, image, pKNN, previousSubImage);
		//imshow("Background Subtraction", subImage);
		thresImage = cam.colorDetection(150, image);
		cam.circlesDetection(image, subImage, thresImage);
		circlesImage = cam.displayCircles(image);
		rectangle(image, cam.getBoundingBoxObs(), Scalar(255, 0, 0), 2, 8, 0);
		imshow("Circles Detection", circlesImage);*/

		/* Strategy 3 : Background subtraction, Hough transformation, markers detection and color recognition */

	}

}
