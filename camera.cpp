#include "camera.h"



Camera::Camera(int numDevice)
{
	// Opening the framegrabber of the camera
	this->cap.open(numDevice);
	if (!cap.isOpened()) {
		cout << "Error opening the framegrapper of the camera";
	}

	this->intrinsicParam = Mat(3, 3, CV_32FC1, double(0));
	this->distortionParam = Mat(1, 5, CV_32FC1, double(0));
}

Camera::~Camera()
{
	// Desallocate the memory and clear the capture
	this->cap.release();
	//delete &(this->intrinsicParam);
	//delete &(this->distortionParam);
}

int Camera::cameraCalib()
{
	// Captured RGB and Gray images
	Mat image;
	Mat acqImageGray;
	
	// Number of images where the chessboard is identified
	int successes = 0;

	// Vector of detected points into the image
	std::vector<Point2f> pointBuf;
	std::vector<std::vector<Point2f> > imagePoints;
	// Reference vector
	std::vector<Point3f> objectBuf;
	std::vector<std::vector<Point3f> > objectPoints;

	// Number of horizontal corners from the reference pattern
	int numHorSquares = 6;
	// Numeber of vertical corners from the reference pattern
	int numVerSquares = 9;
	Size boardSize(numHorSquares, numVerSquares);
	// Reference pattern
	for (int j = 0; j < numHorSquares * numVerSquares; j++)
		objectBuf.push_back(Point3d(j / numHorSquares, j % numHorSquares, 0.0f));

	while (successes < 10) {
		// Capture of one frame in RGB
		cap >> image;
		// Transformation from a RGB image to a gray image
		cvtColor(image, acqImageGray, CV_BGR2GRAY);
		imshow("Gray image", acqImageGray);

		// Search into the captured images the chessboard by pressing SPACE
		if (waitKey(10) == 32) {
			pointBuf.clear();
			int found = 0;

			// Searching chessboard corners
			int flag = CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS;
			found = findChessboardCorners(acqImageGray, boardSize, pointBuf, flag);

			// Display and save the points
			if (found) {
				/*string file = "../data/calibrate" + to_string(successes) + ".jpg";
				imwrite(file, frame);*/
				// Finds the edges in the image
				cornerSubPix(acqImageGray, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				// Display the found corners
				drawChessboardCorners(image, boardSize, Mat(pointBuf), found);
				imshow("Corners detected", image);
				cvWaitKey(10);
				// Save the found corners points
				imagePoints.push_back(pointBuf);
				objectPoints.push_back(objectBuf);
				successes++;
			}
		}

		if (successes == 10) {
			break;
		}

		// Stop capturing images and exit by pressing ESC
		if (waitKey(10) == 27) {
			return(-2);
		}
	}

	cout << "Etalonnage avec: " << successes << " images" << endl;

	// Result of the calibration
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	double rms = 0;

	// Etalonnage caméra
	rms = calibrateCamera(objectPoints, imagePoints, image.size(), this->intrinsicParam, this->distortionParam, rvecs, tvecs);

	if (0.1 < rms && rms < 1) {
		cout << "The calibration went well : " << endl;
		cout << "rms = " << rms << endl;
		cout << "Intrinsic parameters = " << this->intrinsicParam << endl;
		cout << "Distortion parameters = " << this->distortionParam << endl;
		return(0);
	}

	return(-3);
}

int Camera::cameraCalibFromPict()
{
	return(0);
}

int Camera::cameraCorr()
{
	// Rotation matrix
	Mat R;

	Mat map1, map2;
	Mat image;
	Mat rectImage;

	//
	Size sizeImage(this->cap.get(CV_CAP_PROP_FRAME_WIDTH), this->cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	Mat newCameraMatrix = getOptimalNewCameraMatrix(this->intrinsicParam, this->distortionParam, sizeImage, 1, sizeImage);

	// Set paramters to correct distortions
	initUndistortRectifyMap(this->intrinsicParam, this->distortionParam, R, newCameraMatrix, sizeImage, CV_32FC1, map1, map2);

	while (1)
	{
		this->cap >> image; // capture nouvelle image

		remap(image, rectImage, map1, map2, INTER_NEAREST);

		// Affichage de l'image rectifiée
		imshow("Image View", rectImage);

		// Stop capturing images by pressing ESC
		if (waitKey(10) == 27) {
			break;
		}
	}

	return(0);
}
