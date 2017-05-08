/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine and Ishii Hiroyuki
* Mail : sabinecassat@gmail.com
*/

#include "camera.h"

int main(int argc, char** argv) {

	Camera cam(0);

	if (cam.cameraCalib() != 0) {
		cout << "Calibration error" << endl;
	}

	if (!cam.cameraCorr()) {
		cout << "Rectification error" << endl;
	}







	//// Opening the framegrabber of the camera
	//VideoCapture cap(0);
	//if (!cap.isOpened()) {
	//	cout << "Error opening the framegrapper of the camera";
	//	return(-1);
	//}

	//Mat acqImageGray;
	//Mat frame;

	//int successes = 0;
	//// Vector of detected points into the image
	//std::vector<Point2f> pointBuf;
	//std::vector<std::vector<Point2f> > imagePoints;
	//// Reference vector
	//std::vector<Point3f> objectBuf;
	//std::vector<std::vector<Point3f> > objectPoints;

	//// Reference pattern
	//int numHorSquares = 6;
	//int numVerSquares = 9;
	//Size boardSize(numHorSquares, numVerSquares);
	//for (int j = 0; j < numHorSquares * numVerSquares; j++)
	//	objectBuf.push_back(Point3d(j / numHorSquares, j % numHorSquares, 0.0f));

	//while (successes < 10) {
	//	cap >> frame;
	//	cvtColor(frame, acqImageGray, CV_BGR2GRAY);
	//	imshow("Gray image", acqImageGray);

	//	// Search into the captured images the chessboard by pressing SPACE
	//	if (waitKey(10) == 32) {
	//		pointBuf.clear();
	//		int found = 0;

	//		// Finding chessboard corners
	//		int flag = CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS;
	//		found = findChessboardCorners(acqImageGray, boardSize, pointBuf, flag);

	//		// Display and save the points
	//		if (found) {
	//			string file = "../data/calibrate" + to_string(successes) + ".jpg";
	//			imwrite(file, frame);

	//			cornerSubPix(acqImageGray, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	//			drawChessboardCorners(frame, boardSize, Mat(pointBuf), found);
	//			imshow("Corners detected", frame);
	//			cvWaitKey(10);
	//			imagePoints.push_back(pointBuf);
	//			objectPoints.push_back(objectBuf);
	//			successes++;
	//		}
	//	}

	//	if (successes == 10) {
	//		break;
	//	}

	//	// Stop capturing images by pressing ESC
	//	if (waitKey(10) == 27) {
	//		return(-2);
	//	}
	//}

	//cout << "Etalonnage avec: " << successes << " images" << endl;

	//// Varaibles résultat
	//Mat cameraMatrix = Mat(3, 3, CV_32FC1, double(0));
	//Mat distCoeffs;
	//vector<Mat> rvecs;
	//vector<Mat> tvecs;
	//double rms = 0;

	//// Etalonnage caméra
	//rms = calibrateCamera(objectPoints, imagePoints, frame.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

	//std::cout << "rms : " << rms << std::endl;
	//std::cout << "intr : " << cameraMatrix << std::endl;
	//std::cout << "dist : " << distCoeffs << std::endl;


	///***********************************************************/


	//// Rectification
	//Mat R;
	//Mat map1, map2;
	//Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, frame.size(), 1, frame.size());

	//initUndistortRectifyMap(cameraMatrix, distCoeffs, R, newCameraMatrix, frame.size(), CV_32FC1, map1, map2);

	//while (1)
	//{
	//	Mat frame;
	//	cap >> frame; // capture nouvelle image
	//	Mat rectFrame; // Varible pour l'image rectifiée

	//	remap(frame, rectFrame, map1, map2, INTER_NEAREST);

	//	// Affichage de l'image rectifiée
	//	imshow("Image View", rectFrame);

	//	// Stop capturing images by pressing ESC
	//	if (waitKey(10) == 27) break;
	//}

	//// the camera will be deinitialized automatically in VideoCapture destructor
	//return(0);
}
