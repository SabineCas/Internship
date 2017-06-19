#include "algo.h"
#include "interface2.h"

#include <thread>
#include <iostream>

class Algo::Impl {

public:
	std::thread runThr;
	void run();
	bool started;
	bool finished;
	bool close;
	Impl();

	//interface ptr
	Interface * interface;

	//specific stuff;
	int value;
};

Algo::Impl::Impl()
	: started(false)
	, finished(false)
	, close(false)
{}

void Algo::Impl::run()
{
	// Time between two frame in ms
	clock_t dT = clock(), t = clock();

	// Initialization of the camera and the robot
	Camera cam(0);
	cv::Mat image;

	// Calibration of the camera
	if (cam.cameraCalib(false) != 0) {
		std::cout << "Calibration error" << endl;
	}
	double height = 1.73;
	//cam.cameraCorr(height);

	// Infrared LED color
	cv::Scalar IF_lower(100, 34, 60), IF_upper(175, 128, 244);
	AreaClassification classif = AreaClassification();

	// Robot
	Robot robot = Robot(height, cam.getIntrinsicParameters().at<double>(0, 0), cam.getIntrinsicParameters().at<double>(1, 1),
		cam.getIntrinsicParameters().at<double>(0, 2), cam.getIntrinsicParameters().at<double>(1, 2));

	// Kalman filter
	Kalman kalman = Kalman();
	bool found = false, error = false;
	int notFoundCount = 0;

	started = true;

	while (!close) {
		// Update of the timer
		t = clock();

		// Get the next frame
		cam.getCap().read(image);
		if (image.size() == cv::Size(0, 0)) {
			break;
		}

		dT = (clock() - t);
		std::cout << dT << " ";

		// Fix the optical distorsion of the camera
		//remap(image, image, cam.getMap1(), cam.getMap2(), cv::INTER_NEAREST);
		//writer2.write(image);

		// Detecttion of the LED
		cv::cvtColor(image, image, CV_BGR2HSV);
		classif.setInfraredVector(cam.ledDetection(image, IF_lower, IF_upper));
		cv::cvtColor(image, image, CV_HSV2BGR);

		// Classification of the different areas
		classif.updateCurrentFromPrevious();
		classif.updatePreviousFromCurrent();
		classif.updateIdentification(dT);
		classif.identifyLastKnownLocation();
		classif.displayIdentification(image);


		// Update robot position and orientation
		if (classif.getLastKnownBOTTOM().getCoord() != cv::Point(-1, -1) && classif.getLastKnownTOP().getCoord() != cv::Point(-1, -1)) {
			notFoundCount = 0;
			found = true;
			robot.updatePosition(classif.getLastKnownTOP().getCoord(), classif.getLastKnownBOTTOM().getCoord());
			robot.displayImagePosition(image);
			robot.displayImageOrientation(image, classif.getLastKnownTOP().getCoord());


		}
		else {
			notFoundCount++;
			if (notFoundCount >= 50) {
				found = false;
			}
		}

		dT = (clock() - t);
		std::cout << dT << " ";

		// Update Kalman filter
		if (found) {
			kalman.updateMeas(int(robot.getImagePosition().x), int(robot.getImagePosition().y));
			if (classif.getPreviousInfraredVector().empty()) {
				kalman.initiateKalmanFilter();
			}
			else {
				kalman.predictKalmanFilter(dT);
			}
			//kalman.displayEstimatePosition(image);
		}

		dT = (clock() - t);
		std::cout << dT << " ";

		// Update the previous LEDs vector
		classif.setPreviousInfraredVector(classif.getFinalInfraredVector());
		classif.clearFinalInfraredVector();

		//writer3.write(image);
		dT = (clock() - t);
		std::cout << dT << " ";

		cv::cvtColor(image, image, CV_BGR2RGB);
		QImage img(image.data, image.cols, image.rows, QImage::Format_RGB888);
		interface->SetImage(img);

		dT = (clock() - t);
		std::cout << dT << std::endl;

		//std::cout << "value : " << value << std::endl;
		
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));
	interface->end();
	finished = false;
}

Algo::Algo()
{
	impl.reset(new Algo::Impl);
}

void Algo::start()
{
	impl->runThr = std::thread(&Algo::Impl::run, impl.get());
}

void Algo::forceQuit()
{
	if (!impl->started)
		return;
	if (!impl->close)
		impl->close = true;
	if (!impl->finished)
		impl->runThr.join();
}

void Algo::setInterface(Interface *i)
{
	impl->interface = i;
}

void Algo::setValue(int i)
{
	impl->value = i;
}

int Algo::getValue()
{
	return impl->value;
}

