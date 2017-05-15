/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Camera calibration
*/

#include "camera.h"

Camera::Camera(int numDevice)
{
	// Opening the framegrabber of the camera
	this->cap.open(numDevice);
	if (!this->cap.isOpened()) {
		std::cout << "Error opening the framegrapper of the camera" << endl;
	}
	this->intrinsicParam = Mat(3, 3, CV_32FC1, double(0));
	this->distortionParam = Mat(1, 5, CV_32FC1, double(0));
	this->boundingBoxObs = Rect2d(0, 0, this->cap.get(CAP_PROP_FRAME_WIDTH), this->cap.get(CAP_PROP_FRAME_HEIGHT));
}

Camera::~Camera()
{
	// Desallocate the memory and clear the capture
	this->cap.release();
	this->intrinsicParam.release();
	this->distortionParam.release();
	this->map1.release();
	this->map2.release();
}

int Camera::cameraCalib(bool webcam)
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
		if (webcam) {
			// Capture of one frame in RGB
			this->cap >> image;
			// Transformation from a RGB image to a gray image
			cvtColor(image, acqImageGray, CV_BGR2GRAY);
			imshow("Gray image", acqImageGray);
		}
		else {
			string file = "../data/Calibration/calibrate" + to_string(successes) + ".jpg";
			// Read the file
			image = imread(file, CV_LOAD_IMAGE_COLOR);

			// Check for invalid file
			if (!image.data)
			{
				cout << "Could not open or find the image" << std::endl;
				return -1;
			}
			// Transformation from a RGB image to a gray image
			cvtColor(image, acqImageGray, CV_BGR2GRAY);
			imshow("Gray image", acqImageGray);
		}

		// Search into the captured images the chessboard by pressing SPACE
		if (waitKey(10) == 32 || !webcam) {
			pointBuf.clear();
			int found = 0;

			// Searching chessboard corners
			int flag = CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS;
			found = findChessboardCorners(acqImageGray, boardSize, pointBuf, flag);

			// Display and save the points
			if (found) {
				/*string file = "../data/calibrate" + to_string(successes) + ".jpg";
				imwrite(file, image);*/
				// Finds the edges in the image
				cornerSubPix(acqImageGray, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				// Display the found corners
				drawChessboardCorners(image, boardSize, Mat(pointBuf), found);
				imshow("Corners detected", image);
				string file = "../data/Calibration/process" + to_string(successes) + ".jpg";
				imwrite(file, image);
				cvWaitKey(10);
				// Save the found corners points
				imagePoints.push_back(pointBuf);
				objectPoints.push_back(objectBuf);
				successes++;
			}
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
		destroyAllWindows();
		return(0);
	}

	return(-3);
}

int Camera::cameraCorr()
{
	// Rotation matrix
	Mat R;

	Mat map1, map2;
	Mat image;
	Mat rectImage;

	// Determine the size of the frame
	Size sizeImage(this->cap.get(CV_CAP_PROP_FRAME_WIDTH), this->cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	Mat newCameraMatrix = getOptimalNewCameraMatrix(this->intrinsicParam, this->distortionParam, sizeImage, 1, sizeImage);

	// Set paramters to correct distortions
	initUndistortRectifyMap(this->intrinsicParam, this->distortionParam, R, newCameraMatrix, sizeImage, CV_32FC1, this->map1, this->map2);

	return(0);
}

Mat Camera::subtractionBack(int solution, Mat image1, Ptr<BackgroundSubtractor> pKNN, Mat previousSubImage)
{
	Mat subImage;

	if (solution == 1) {
		/*********************** Solution 1 : Background subtraction ***********************/
		Mat image2;

		// Capturing the image to compare with the previous frame
		waitKey(50);
		this->cap >> image2;

		// Transformation from a RGB image to a gray image
		cv::cvtColor(image1, image1, CV_BGR2GRAY);
		cv::cvtColor(image2, image2, CV_BGR2GRAY);

		// Gaussian filter
		GaussianBlur(image1, image1, cv::Size(3, 3), 0);
		GaussianBlur(image2, image2, cv::Size(3, 3), 0);

		// Subtraction
		absdiff(image1, image2, subImage);

		// Binarisation
		threshold(subImage, subImage, SENSITIVITY_VALUE, 255, CV_THRESH_BINARY);

		// Morphological processing
		erode(subImage, subImage, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
		dilate(subImage, subImage, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

		return(subImage);
	}
	else if (solution == 2) {

		/*********************** Solution 2 : Background subtraction ***********************/
		pKNN->apply(image1, subImage);
		// Morphological processing
		erode(subImage, subImage, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
		dilate(subImage, subImage, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

		// Reduce the noise so we avoid false circle detection with a Guassian filter
		GaussianBlur(subImage, subImage, Size(5, 5), 1, 1);

		// Binarisation
		threshold(subImage, subImage, SENSITIVITY_VALUE, 255, CV_THRESH_BINARY);

		return(subImage);
	}
	else {
		return(subImage);
	}
}

void Camera::circlesDetection(Mat image, Mat subImage)
{
	// Variable used for edges detection
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	RNG rng(12345);
	Mat grayDrawing;

	cv::Range width(this->boundingBoxObs.x, this->boundingBoxObs.x + this->boundingBoxObs.width);
	cv::Range height(this->boundingBoxObs.y, this->boundingBoxObs.y + this->boundingBoxObs.height);
	subImage(height, width).copyTo(subImage);
	//imshow("tests", subImage);

	// Find the edges of each different area
	findContours(subImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	Mat drawing = Mat::zeros(subImage.size(), CV_8UC3);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	// std::vector<std::vector<cv::Point>> convexHulls(contours.size());

	// Draw the edges
	for (size_t i = 0; i < contours.size(); i++)
	{
		// If the area is too big or too small, we reject this area as the real robot
		if (contourArea(contours[i], false) < 500 || contourArea(contours[i], false) > 50000) {
			// Calculate the convex hull fo each different area
			/*convexHull(contours[i], convexHulls[i]);
			drawContours(drawing, convexHulls, (int)i, color, 2, 8, hierarchy, 0, Point());*/
			contours.erase(contours.begin() + i);
		}
		else {
			drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
		}
	}

	// Circle Hough Detection
	cvtColor(drawing, grayDrawing, CV_BGR2GRAY);
	HoughCircles(grayDrawing, this->circles, CV_HOUGH_GRADIENT, 2, drawing.rows / 2, 200, 100);

	cout << this->circles.size() << endl;

	// If no circle is detected or too many circles are detected, we suppose that we lost the
	// robot because of too many changement inside the picture, and so keep the previous data
	if (this->circles.size() == 0 || this->circles.size() > 3) {
		this->circles.clear();
		this->circles.reserve(this->previousCircles.size());
		copy(this->previousCircles.begin(), this->previousCircles.end(), back_inserter(this->circles));
		wideringBoundingBox(this->cap.get(CAP_PROP_FRAME_WIDTH) / 8);
		this->detectedCircle = false;
	}
	else {
		this->previousCircles.clear();
		this->previousCircles.reserve(this->circles.size());
		copy(this->circles.begin(), this->circles.end(), back_inserter(this->previousCircles));
		this->detectedCircle = true;
	}
}

Mat Camera::displayCircles(Mat image)
{
	int maxX = 0, maxY = 0, minX = image.size().width, minY = image.size().height;

	if (this->circles.size() > 0) {
		// We use the first circles of the output list from the HoughCircles function because
		// this list is sorted according to the vote in the accumulator in descending order

		// Because we limited the search area to the boundingBoxObs, we have to adjust the coordinate of the circle inside the all picture
		int X = cvRound(this->circles[0][0] + this->boundingBoxObs.x);
		int Y = cvRound(this->circles[0][1] + this->boundingBoxObs.y);
		Point center(X, Y);
		int radius = cvRound(this->circles[0][2]);

		// Draw the circle center on the Mat image
		circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// Draw the circle outline on the Mat image
		circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);

		if (maxX < cvRound(X + radius)) {
			maxX = cvRound(X + radius);
		}
		if (maxY < cvRound(Y + radius)) {
			maxY = cvRound(Y + radius);
		}
		if (minX > cvRound(X - radius)) {
			minX = cvRound(X - radius);
		}
		if (minY > cvRound(Y - radius)) {
			minY = cvRound(Y - radius);
		}

		if (this->detectedCircle) {
			// Set the boounding box on the image where we will search the robot in the next frame
			this->setBoundingBoxObs(minX, maxX, minY, maxY);
		}
	}

	return(image);
}

VideoCapture Camera::getCap()
{
	return this->cap;
}

Rect2d Camera::getBoundingBoxObs()
{
	return this->boundingBoxObs;
}

Mat Camera::getMap1()
{
	return this->map1;
}

Mat Camera::getMap2()
{
	return this->map2;
}

void Camera::setBoundingBoxObs(int minX, int maxX, int minY, int maxY)
{
	if (maxX > this->cap.get(CAP_PROP_FRAME_WIDTH)) {
		maxX = this->cap.get(CAP_PROP_FRAME_WIDTH);
	}
	if (maxY > this->cap.get(CAP_PROP_FRAME_HEIGHT)) {
		maxY = this->cap.get(CAP_PROP_FRAME_HEIGHT);
	}
	if (minX < 0) {
		minX = 0;
	}
	if (minY < 0) {
		minY = 0;
	}
	this->boundingBoxObs.height = abs(maxY - minY);
	this->boundingBoxObs.width = abs(maxX - minX);
	this->boundingBoxObs.x = minX;
	this->boundingBoxObs.y = minY;
}

void Camera::wideringBoundingBox(int value)
{
	int minX = this->boundingBoxObs.x - value;
	int maxX = this->boundingBoxObs.x + this->boundingBoxObs.width + value;
	int minY = this->boundingBoxObs.y - value;
	int maxY = this->boundingBoxObs.y + this->boundingBoxObs.height + value;
	// Setting the new configuration to the bounding box
	this->setBoundingBoxObs(minX, maxX, minY, maxY);
}
