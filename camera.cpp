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
	//this->cap.open("../data/TestLED.mp4");
	this->cap.open(numDevice);
	if (!this->cap.isOpened()) {
		std::cout << "Error opening the framegrapper of the camera" << std::endl;
	}
	this->intrinsicParam = cv::Mat(3, 3, CV_32FC1, double(0));
	this->distortionParam = cv::Mat(1, 5, CV_32FC1, double(0));
	this->boundingBoxObs = cv::Rect2d(0, 0, this->cap.get(cv::CAP_PROP_FRAME_WIDTH), this->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
	this->circles.push_back(cv::Vec3f(0, 0, 0));
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
	cv::Mat image, acqImageGray;

	// Number of images where the chessboard is identified
	int successes = 0;

	// Vector of detected points into the image
	std::vector<cv::Point2f> pointBuf;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	// Reference vector
	std::vector<cv::Point3f> objectBuf;
	std::vector<std::vector<cv::Point3f> > objectPoints;

	// Number of horizontal corners from the reference pattern
	int numHorSquares = 6;
	// Numeber of vertical corners from the reference pattern
	int numVerSquares = 9;
	cv::Size boardSize(numHorSquares, numVerSquares);
	// Reference pattern
	for (int j = 0; j < numHorSquares * numVerSquares; j++)
		objectBuf.push_back(cv::Point3d(j / numHorSquares, j % numHorSquares, 0.0f));

	while (successes < 10) {
		if (webcam) {
			// Capture of one frame in RGB
			this->cap >> image;
			// Transformation from a RGB image to a gray image
			cvtColor(image, acqImageGray, CV_BGR2GRAY);
			//imshow("Gray image", acqImageGray);
		}
		else {
			std::string file = "../data/Calibration/calibrate" + std::to_string(successes) + ".jpg";
			// Read the file
			image = cv::imread(file, CV_LOAD_IMAGE_COLOR);

			// Check for invalid file
			if (!image.data)
			{
				std::cout << "Could not open or find the image" << std::endl;
				return -1;
			}
			// Transformation from a RGB image to a gray image
			cvtColor(image, acqImageGray, CV_BGR2GRAY);
			//imshow("Gray image", acqImageGray);
		}

		// Search into the captured images the chessboard by pressing SPACE
		if (cv::waitKey(10) == 32 || !webcam) {
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
				cornerSubPix(acqImageGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				// Display the found corners
				drawChessboardCorners(image, boardSize, cv::Mat(pointBuf), found);
				//imshow("Corners detected", image);
				std::string file = "../data/Calibration/process" + std::to_string(successes) + ".jpg";
				imwrite(file, image);
				cvWaitKey(10);
				// Save the found corners points
				imagePoints.push_back(pointBuf);
				objectPoints.push_back(objectBuf);
				successes++;
			}
		}

		// Stop capturing images and exit by pressing ESC
		if (cv::waitKey(10) == 27) {
			return(-2);
		}
	}

	std::cout << "Etalonnage avec: " << successes << " images" << std::endl;

	// Result of the calibration
	std::vector<cv::Mat> rvecs, tvecs;
	double rms = 0;

	// Etalonnage caméra
	rms = calibrateCamera(objectPoints, imagePoints, image.size(), this->intrinsicParam, this->distortionParam, rvecs, tvecs);

	if (0.1 < rms && rms < 1) {
		std::cout << "The calibration went well : " << std::endl;
		std::cout << "rms = " << rms << std::endl;
		std::cout << "Intrinsic parameters = " << this->intrinsicParam << std::endl;
		std::cout << "Distortion parameters = " << this->distortionParam << std::endl;
		cv::destroyAllWindows();
		return(0);
	}

	return(-3);
}

int Camera::cameraCorr()
{
	cv::Mat R, map1, map2, image, rectImage;

	// Determine the size of the frame
	cv::Size sizeImage((int)this->cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)this->cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	cv::Mat newCameraMatrix = getOptimalNewCameraMatrix(this->intrinsicParam, this->distortionParam, sizeImage, 1, sizeImage);

	// Set paramters to correct distortions
	initUndistortRectifyMap(this->intrinsicParam, this->distortionParam, R, newCameraMatrix, sizeImage, CV_32FC1, this->map1, this->map2);

	return(0);
}

cv::Mat Camera::updateBackground(cv::Mat image, cv::Ptr<cv::BackgroundSubtractor> pKNN)
{
	cv::Mat subImage;
	pKNN->apply(image, subImage);
	return(subImage);
}

cv::Mat Camera::improveBackSubtr(cv::Mat subImage)
{
	// Morphological processing
	cv::erode(subImage, subImage, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
	cv::dilate(subImage, subImage, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

	// Reduce the noise so we avoid false circle detection with a Guassian filter
	cv::GaussianBlur(subImage, subImage, cv::Size(5, 5), 1, 1);

	// Binarisation
	cv::threshold(subImage, subImage, SENSITIVITY_VALUE, 255, CV_THRESH_BINARY);

	return(subImage);
}

cv::Mat Camera::colorDetection(cv::Mat image, cv::Scalar lower, cv::Scalar upper)
{
	//cv::Mat image2 = cv::imread("../data/data.png");
	cv::cvtColor(image, image, CV_BGR2HSV);

	/*cv::Mat hsvChannels[3];
	double minVal, maxVal;
	cv::split(hsv, hsvChannels);
	cv::minMaxLoc(hsvChannels[0], &minVal, &maxVal);
	std::cout << "Hue: Min = " << minVal << ", Max = " << maxVal << std::endl;
	cv::minMaxLoc(hsvChannels[1], &minVal, &maxVal);
	std::cout << "Saturation: Min = " << minVal << ", Max = " << maxVal << std::endl;
	cv::minMaxLoc(hsvChannels[2], &minVal, &maxVal);
	std::cout << "Value: Min = " << minVal << ", Max = " << maxVal << std::endl;
	cv::waitKey(0);*/

	// Isolate every area that have the color of the LED
	cv::inRange(image, lower, upper, image);
	return image;
}

std::vector<infraredLight> Camera::ledDetection(cv::Mat image, cv::Scalar lower, cv::Scalar upper)
{
	// Variable used for edges detection
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	// Vector of every potential light from the LEDs
	std::vector<infraredLight> ledVector;
	int cpt = 0;

	// DEBUG
	/*image = cv::imread("../data/InfraredLED.PNG");
	cv::Mat hsv;
	cv::cvtColor(image, hsv, CV_BGR2HSV);
	cv::Mat hsvChannels[3];
	double minVal, maxVal;
	cv::split(hsv, hsvChannels);
	cv::minMaxLoc(hsvChannels[0], &minVal, &maxVal);
	std::cout << "Hue: Min = " << minVal << ", Max = " << maxVal << std::endl;
	cv::minMaxLoc(hsvChannels[1], &minVal, &maxVal);
	std::cout << "Saturation: Min = " << minVal << ", Max = " << maxVal << std::endl;
	cv::minMaxLoc(hsvChannels[2], &minVal, &maxVal);
	std::cout << "Value: Min = " << minVal << ", Max = " << maxVal << std::endl;
	cv::waitKey(0);*/

	// Gaussian filter
	//cv::GaussianBlur(image, image, cv::Size(5, 5), 1, 1);

	// Isolate every area that have the color of the LED
	cv::inRange(image, lower, upper, image);
	// Morphological processing
	cv::dilate(image, image, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
	cv::morphologyEx(image, image, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(2, 2)));
	cv::morphologyEx(image, image, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(2, 2)));

	//cv::imshow("LED", image);

	// Find the edges of each different area
	findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// Save the center of each area of the image
	for (std::vector<std::vector<cv::Point>>::size_type i = 0; i < contours.size(); i++) {
		if (contourArea(contours[i], false) > minimumSizeAreaLight && contourArea(contours[i], false) < maximumSizeAreaLight) {
			int maxX = 0, maxY = 0, minX = image.size().width, minY = image.size().height;
			for (std::vector<cv::Point>::size_type j = 0; j < contours[i].size(); j++) {
				if (maxX < contours[i][j].x) {
					maxX = contours[i][j].x;
				}
				if (maxY < contours[i][j].y) {
					maxY = contours[i][j].y;
				}
				if (minX > contours[i][j].x) {
					minX = contours[i][j].x;
				}
				if (minY > contours[i][j].y) {
					minY = contours[i][j].y;
				}
			}
			ledVector.push_back(infraredLight(true, cv::Point((maxX + minX) / 2, (maxY + minY) / 2), cpt, 0, 0, contourArea(contours[i], false), "UNKNOWN"));
			cpt++;
		}
	}

	// Merge close areas
	std::vector<std::vector<infraredLight>> areas;
	std::vector<infraredLight> ledFinalVector, temp;
	bool proche_voisin = false;
	cpt = 0;

	while (!(ledVector.empty())) {
		temp.clear();
		temp.push_back(ledVector[0]);
		areas.push_back(temp);
		ledVector.erase(ledVector.begin());

		for (std::vector<infraredLight>::size_type i = 0; i < ledVector.size(); i++) {
			if (ledVector[i].areClose(areas[cpt][0].getCoord())) {
				areas[cpt].push_back(ledVector[i]);
				ledVector.erase(ledVector.begin() + i);
			}
		}

		/*proche_voisin = true;
		while (proche_voisin) {
			proche_voisin = false;
			for (std::vector<LightArea>::size_type i = 0; i < ledVector.size(); i++) {
				for (std::vector<LightArea>::size_type j = 0; j < areas[cpt].size(); j++) {
					if (ledVector[i].areClose(areas[cpt][j].getCoord())) {
						proche_voisin = true;
						ledVector.erase(ledVector.begin() + i);
						break;
					}
				}
			}
		}*/

		int x = 0, y = 0, size = 0;
		for (std::vector<cv::Point>::size_type i = 0; i < areas[0].size(); i++) {
			x += areas[cpt][i].getCoord().x;
			y += areas[cpt][i].getCoord().y;
			size += areas[cpt][i].getSizeArea();
		}
		x = (int)(x / areas[cpt].size());
		y = (int)(y / areas[cpt].size());
		areas[cpt].clear();
		areas[cpt].push_back(infraredLight(true, cv::Point(x, y), cpt, 0, 0, size, "UNKNOWN"));
		ledFinalVector.push_back(infraredLight(true, cv::Point(x, y), cpt, 0, 0, size, "UNKNOWN"));
		cpt++;
	}

	return(ledFinalVector);
}

int Camera::ledFrequency(clock_t time, bool detected, bool previousDetected)
{
	int time1 = 100, time2 = 200;
	if (detected && !previousDetected) {
		LEDTimeON = time;
		if (LEDTimeOFF <= time1 && LEDTimeOFF > 0) {
			return(1);
		}
		else {
			return(2);
		}
	}
	else if (detected && previousDetected) {
		LEDTimeON += time;
		if (LEDTimeOFF <= time1 && LEDTimeOFF > 0) {
			return(1);
		}
		else {
			return(2);
		}
	}
	else if (!detected && previousDetected) {
		LEDTimeOFF = time;
		if (LEDTimeON <= time1 && LEDTimeOFF > 0) {
			return(1);
		}
		else {
			return(2);
		}
	}
	else {
		LEDTimeOFF += time;
		if (LEDTimeON <= time1 && LEDTimeON > 0) {
			return(1);
		}
		else {
			return(2);
		}
	}
}

bool Camera::evaluateMarkersPosition(std::vector<cv::Point> blueVector)
{
	this->nbDetectedLED = 0;
	if (this->detectedBlueLED) {
		this->nbDetectedLED++;
	}

	if (this->nbDetectedLED > 2) {
		// Check if the positionning of the LED are coherent with the mechanical configuration of the robot
		return(true);
	}
	return(false);
}

void Camera::circlesDetection(cv::Mat subImage)
{
	// Variable used for edges detection
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::RNG rng(12345);
	cv::Mat grayDrawing;

	cv::Range width((int)this->boundingBoxObs.x, (int)(this->boundingBoxObs.x + this->boundingBoxObs.width));
	cv::Range height((int)this->boundingBoxObs.y, (int)(this->boundingBoxObs.y + this->boundingBoxObs.height));
	subImage(height, width).copyTo(subImage);
	//imshow("tests", subImage);

	// Find the edges of each different area
	findContours(subImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	cv::Mat drawing = cv::Mat::zeros(subImage.size(), CV_8UC3);
	cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

	// Draw the edges
	for (size_t i = 0; i < contours.size(); i++)
	{
		// If the area is too big or too small, we reject this area as the real robot
		if (contourArea(contours[i], false) < 5000 || contourArea(contours[i], false) > 50000) {
			contours.erase(contours.begin() + i);
		}
		else {
			drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point());
		}
	}

	std::vector<cv::Vec3f> temp;

	// Circle Hough Detection
	cvtColor(drawing, grayDrawing, CV_BGR2GRAY);
	HoughCircles(grayDrawing, temp, CV_HOUGH_GRADIENT, 2, drawing.rows / 2, 200, 100);

	// If no circle is detected or too many circles are detected, we suppose that we lost the
	// robot because of too many changement inside the picture, and so keep the previous data
	if (temp.size() == 1) {
		this->circles.clear();
		this->circles.reserve(temp.size());
		copy(temp.begin(), temp.end(), back_inserter(this->circles));
		this->detectedCircle = true;
	}
	else if (this->circles.size() > 0) {
		if (this->boundingBoxObs.x - WIDE_BOUNDING_BOX_X < 0) {
			this->circles[0][0] = this->circles[0][0] + WIDE_BOUNDING_BOX_X - (int)abs(this->boundingBoxObs.x - WIDE_BOUNDING_BOX_X);
		}
		else {
			this->circles[0][0] = this->circles[0][0] + WIDE_BOUNDING_BOX_X;
		}
		if (this->boundingBoxObs.y - WIDE_BOUNDING_BOX_X < 0) {
			this->circles[0][1] = this->circles[0][1] + WIDE_BOUNDING_BOX_X - (int)abs(this->boundingBoxObs.y - WIDE_BOUNDING_BOX_X);
		}
		else {
			this->circles[0][1] = this->circles[0][1] + WIDE_BOUNDING_BOX_X;
		}
		this->detectedCircle = false;
	}
}

cv::Point Camera::coherenceCirclesMarkers(std::vector<cv::Point> blueVector)
{
	int radius = 1000;
	if (!(blueVector.empty())) {
		for (std::vector<cv::Point>::size_type i = 0; i < blueVector.size(); i++) {
			bool ok = sqrt((blueVector[i].x - this->circles[0][0]) * (blueVector[i].x - this->circles[0][0]) + (blueVector[i].y - this->circles[0][1]) * (blueVector[i].y - this->circles[0][1])) <= this->circles[0][2] + radius;
			if (ok) {
				return(blueVector[i]);
			}
		}
	}
	cv::Point temp(-1, -1);
	return(temp);
}

cv::Mat Camera::displayCircles(cv::Mat image)
{
	int maxX = 0, maxY = 0, minX = image.size().width, minY = image.size().height;

	// We use the first circles of the output list from the HoughCircles function because
	// this list is sorted according to the vote in the accumulator in descending order
	if (this->circles.size() > 0) {
		// Because we limited the search area to the boundingBoxObs, we have to adjust the coordinate of the circle inside the all picture
		int X = cvRound(this->circles[0][0] + this->boundingBoxObs.x);
		int Y = cvRound(this->circles[0][1] + this->boundingBoxObs.y);

		cv::Point center(X, Y);
		int radius = cvRound(this->circles[0][2]);

		// Draw the circle center on the Mat image
		//circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		// Draw the circle outline on the Mat image
		//circle(image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

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
			// Set the boounding box position and size on the image where we will search the robot in the next frame
			this->setBoundingBoxObs(minX, maxX, minY, maxY);
			this->circles[0][0] = X - (float)this->boundingBoxObs.x;
			this->circles[0][1] = Y - (float)this->boundingBoxObs.y;
		}
		else {
			// Widen the size of the searching bounding box area
			//wideringBoundingBox(WIDE_BOUNDING_BOX_X);
		}
	}
	return(image);
}

void Camera::wideringBoundingBox(int value)
{
	// Widen the bounding box with the given value argument
	int minX = (int)this->boundingBoxObs.x - value;
	int maxX = (int)this->boundingBoxObs.x + (int)this->boundingBoxObs.width + value;
	int minY = (int)this->boundingBoxObs.y - value;
	int maxY = (int)this->boundingBoxObs.y + (int)this->boundingBoxObs.height + value;

	// Setting the new configuration to the bounding box
	this->setBoundingBoxObs(minX, maxX, minY, maxY);
}

cv::VideoCapture Camera::getCap()
{
	return this->cap;
}

cv::Rect2d Camera::getBoundingBoxObs()
{
	return this->boundingBoxObs;
}

cv::Mat Camera::getMap1()
{
	return this->map1;
}

cv::Mat Camera::getMap2()
{
	return this->map2;
}

bool Camera::getDetectedCircle()
{
	return detectedCircle;
}

bool Camera::getDetectedBlueLED()
{
	return this->detectedBlueLED;
}

int Camera::getNbDetectedLED()
{
	return this->nbDetectedLED;
}

clock_t Camera::getLEDTimeOFF()
{
	return (LEDTimeOFF);
}

clock_t Camera::getLEDTimeON()
{
	return (LEDTimeON);
}

void Camera::setBoundingBoxObs(int minX, int maxX, int minY, int maxY)
{
	// If minX, maxX, minY and maxY values are outside the frame, we set them at the limit value of the frame
	if (maxX > this->cap.get(cv::CAP_PROP_FRAME_WIDTH)) {
		maxX = (int)this->cap.get(cv::CAP_PROP_FRAME_WIDTH);
	}
	if (maxY > this->cap.get(cv::CAP_PROP_FRAME_HEIGHT)) {
		maxY = (int)this->cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	}
	if (minX < 0) {
		minX = 0;
	}
	if (minY < 0) {
		minY = 0;
	}
	// Update the searching bounding box area
	this->boundingBoxObs.height = abs(maxY - minY);
	this->boundingBoxObs.width = abs(maxX - minX);
	this->boundingBoxObs.x = minX;
	this->boundingBoxObs.y = minY;
}

void Camera::setDetectedCircle(bool det)
{
	this->detectedCircle = det;
}


