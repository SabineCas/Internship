/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Qt interface
*/

#include <windows.h>
#include "interface2.h"
#include "algo.h"

MainInterface::MainInterface(int argc, char * argv[]) : app(argc, argv)
{
	this->create();
}

int MainInterface::run()
{
	// Display the window
	this->window.show();
	// Display the label contained in the window
	this->label->show();
	// Run the application
	int res = this->app.exec();
	this->algo->forceQuit();
	return res;
}

void MainInterface::create()
{
	// Create the window
	window.setWindowTitle("Spherical Robot for Child Care");
	// Place the window
	window.setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, window.size(),
		QRect(QPoint(10, 50), QPoint(window.width() + 10, window.height() + 50))));
	// Insert layouts in the window
	mainLayout = new QVBoxLayout();
	hboxLayout = new QHBoxLayout(&window);

	// Initialize the variables of the interface
	textResolution = new QLabel("Resolution of the camera : ");
	textDistanceArea = new QLabel("Maximum distance between two pixels belonging to the same area : ");
	textNbRobot = new QLabel("Maximum amount of robots that can appear on the camera pictures : ");
	textDisplayOpt = new QLabel("Display options : ");
	textHeight = new QLabel("Height distance between the camera and the ground (in centimeters) : ");
	textfreqLED1 = new QLabel("Blinking frequency of the LED1 (in milliseconds) : ");
	textfreqLED2 = new QLabel("Blinking frequency of the LED2 (in milliseconds) : ");
	textMotorR = new QLabel("Gain of the right motor (between 1 and 31) which will be used in debug mode : ");
	textMotorL = new QLabel("Gain of the left motor (between 1 and 31) which will be used in debug mode : ");
	label = new ClickableLabel();
	label->setAlignment(Qt::AlignTop);

	checkPosition = new QCheckBox("Display the position of the robot : ");
	checkOrientation = new QCheckBox("Display the orientation of the robot : ");
	checkIdentification = new QCheckBox("Display the identification of each region that can be recognized as a LED of the robot : ");
	checkKalman = new QCheckBox("Display the estimated position of the robot (Kalman) : ");
	buttonCommand = new QCheckBox("Send commands to the robot");
	debug = new QCheckBox("Setting the debug mode : ");
	loadGainFile = new QCheckBox("Load the motor gains for each command from the file ../data/MotorGain/gain.txt : ");

	spinBoxDist = new QSpinBox;
	spinBoxDist->setValue(30);
	sliderDist = new QSlider(Qt::Horizontal);
	sliderDist->setValue(30);
	spinBoxRobot = new QSpinBox;
	spinBoxRobot->setValue(1);
	sliderRobot = new QSlider(Qt::Horizontal);
	sliderRobot->setValue(1);
	spinBoxHeight = new QSpinBox;
	spinBoxHeight->setMaximum(500);
	spinBoxHeight->setValue(170);
	sliderHeight = new QSlider(Qt::Horizontal);
	sliderHeight->setMaximum(500);
	sliderHeight->setValue(170);
	spinBoxFreqLED1 = new QSpinBox;
	spinBoxFreqLED1->setMaximum(500);
	spinBoxFreqLED1->setValue(125);
	sliderFreqLED1 = new QSlider(Qt::Horizontal);
	sliderFreqLED1->setMaximum(500);
	sliderFreqLED1->setValue(125);
	spinBoxFreqLED2 = new QSpinBox;
	spinBoxFreqLED2->setMaximum(500);
	spinBoxFreqLED2->setValue(250);
	sliderFreqLED2 = new QSlider(Qt::Horizontal);
	sliderFreqLED2->setMaximum(500);
	sliderFreqLED2->setValue(250);

	spinBoxMotor1 = new QSpinBox;
	spinBoxMotor1->setMaximum(32);
	spinBoxMotor1->setMinimum(1);
	spinBoxMotor1->setValue(14);
	sliderMotor1 = new QSlider(Qt::Horizontal);
	sliderMotor1->setMaximum(32);
	sliderMotor1->setMinimum(1);
	sliderMotor1->setValue(14);
	spinBoxMotor2 = new QSpinBox;
	spinBoxMotor2->setMaximum(32);
	spinBoxMotor2->setMinimum(1);
	spinBoxMotor2->setValue(14);
	sliderMotor2 = new QSlider(Qt::Horizontal);
	sliderMotor2->setMaximum(32);
	sliderMotor2->setMinimum(1);
	sliderMotor2->setValue(14);

	comboBox = new QComboBox();

	// Add the previously created widgets to the layouts 
	hboxLayout->addWidget(label);
	hboxLayout->addLayout(mainLayout);

	// Setting the sending of the command
	mainLayout->addWidget(buttonCommand);

	// Setting the frequency of the LEDs
	mainLayout->addWidget(textfreqLED1);
	mainLayout->addWidget(spinBoxFreqLED1);
	mainLayout->addWidget(sliderFreqLED1);
	mainLayout->addWidget(textfreqLED2);
	mainLayout->addWidget(spinBoxFreqLED2);
	mainLayout->addWidget(sliderFreqLED2);

	// Setting the resolution of the camera
	mainLayout->addWidget(textResolution);
	comboBox->addItem("640 x 480");
	comboBox->addItem("1280 x 720");
	comboBox->addItem("1920 x 1080");
	comboBox->setCurrentIndex(1);
	mainLayout->addWidget(comboBox);

	// Setting the displayed information
	mainLayout->addWidget(textDisplayOpt);
	mainLayout->addWidget(checkPosition);
	mainLayout->addWidget(checkOrientation);
	mainLayout->addWidget(checkIdentification);
	mainLayout->addWidget(checkKalman);

	// Distance between two pixels to belong to the same area
	mainLayout->addWidget(textDistanceArea);
	mainLayout->addWidget(spinBoxDist);
	mainLayout->addWidget(sliderDist);

	// Amount of robots
	mainLayout->addWidget(textNbRobot);
	mainLayout->addWidget(spinBoxRobot);
	mainLayout->addWidget(sliderRobot);

	// Height of the camera
	mainLayout->addWidget(textHeight);
	mainLayout->addWidget(spinBoxHeight);
	mainLayout->addWidget(sliderHeight);

	// Debug mode
	mainLayout->addWidget(debug);
	mainLayout->addWidget(loadGainFile);

	// Gain of the motors
	mainLayout->addWidget(textMotorR);
	mainLayout->addWidget(spinBoxMotor1);
	mainLayout->addWidget(sliderMotor1);
	mainLayout->addWidget(textMotorL);
	mainLayout->addWidget(spinBoxMotor2);
	mainLayout->addWidget(sliderMotor2);

	// Create an handle function for each button, checkbox, slider, ...
	// Some of them have a direct link with a function belonging to the instance of the detection algorithm
	QObject::connect(label, &ClickableLabel::clicked, std::bind(&Algo::setDesiredPoint, std::ref(this->algo), std::placeholders::_1, std::placeholders::_2));
	QObject::connect(spinBoxDist, SIGNAL(valueChanged(int)), sliderDist, SLOT(setValue(int)));
	QObject::connect(sliderDist, SIGNAL(valueChanged(int)), spinBoxDist, SLOT(setValue(int)));
	QObject::connect(sliderDist, &QSlider::valueChanged, std::bind(&Algo::setDistanceAreaLight, std::ref(this->algo), std::placeholders::_1));

	QObject::connect(spinBoxFreqLED1, SIGNAL(valueChanged(int)), sliderFreqLED1, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED1, SIGNAL(valueChanged(int)), spinBoxFreqLED1, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED1, &QSlider::valueChanged, std::bind(&Algo::setFreqLED1, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(spinBoxFreqLED2, SIGNAL(valueChanged(int)), sliderFreqLED2, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED2, SIGNAL(valueChanged(int)), spinBoxFreqLED2, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED2, &QSlider::valueChanged, std::bind(&Algo::setFreqLED2, std::ref(this->algo), std::placeholders::_1));

	QObject::connect(spinBoxRobot, SIGNAL(valueChanged(int)), sliderRobot, SLOT(setValue(int)));
	QObject::connect(sliderRobot, SIGNAL(valueChanged(int)), spinBoxRobot, SLOT(setValue(int)));
	QObject::connect(sliderRobot, &QSlider::valueChanged, std::bind(&Algo::setNbRobot, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(spinBoxHeight, SIGNAL(valueChanged(int)), sliderHeight, SLOT(setValue(int)));
	QObject::connect(sliderHeight, SIGNAL(valueChanged(int)), spinBoxHeight, SLOT(setValue(int)));
	QObject::connect(sliderHeight, &QSlider::valueChanged, std::bind(&Algo::setHeight, std::ref(this->algo), std::placeholders::_1));

	QObject::connect(spinBoxMotor1, SIGNAL(valueChanged(int)), sliderMotor1, SLOT(setValue(int)));
	QObject::connect(sliderMotor1, SIGNAL(valueChanged(int)), spinBoxMotor1, SLOT(setValue(int)));
	QObject::connect(sliderMotor1, &QSlider::valueChanged, std::bind(&Algo::setGainMotor1, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(spinBoxMotor2, SIGNAL(valueChanged(int)), sliderMotor2, SLOT(setValue(int)));
	QObject::connect(sliderMotor2, SIGNAL(valueChanged(int)), spinBoxMotor2, SLOT(setValue(int)));
	QObject::connect(sliderMotor2, &QSlider::valueChanged, std::bind(&Algo::setGainMotor2, std::ref(this->algo), std::placeholders::_1));

	void (QComboBox::*indexChangedSignal)(int) = &QComboBox::currentIndexChanged;
	QObject::connect(comboBox, indexChangedSignal, std::bind(&MainInterface::translateComboBox, this, std::placeholders::_1));
	QObject::connect(checkPosition, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayPosition, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkOrientation, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayOrientation, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkIdentification, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayIdentification, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkKalman, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayKalman, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(buttonCommand, &QCheckBox::stateChanged, std::bind(&Algo::sendCommand, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(debug, &QCheckBox::stateChanged, std::bind(&Algo::setDebug, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(loadGainFile, &QCheckBox::stateChanged, std::bind(&Algo::loadGainFile, std::ref(this->algo), std::placeholders::_1));
}

void MainInterface::end()
{
	this->app.exit();
}

void MainInterface::setDistance(int i)
{
	this->spinBoxDist->setValue(i);
}

void MainInterface::SetImage(const QImage & image)
{
	QPixmap p;
	p.convertFromImage(image);
	this->label->setPixmap(p);
}

void MainInterface::translateComboBox(int cbb)
{
	switch (cbb) {
	case 0:
		this->algo->setResolution(640, 480);
		break;
	case 1:
		this->algo->setResolution(1280, 720);
		break;
	case 2:
		this->algo->setResolution(1920, 1080);
		break;
	default:
		this->algo->setResolution(640, 480);
	}
}

void MainInterface::setAlgo(Algo *a)
{
	this->algo = a;
}
