#include <windows.h>
#include "interface2.h"

#include "algo.h"

MainInterface::MainInterface(int argc, char * argv[]) : app(argc, argv)
{
	this->create();
}

int MainInterface::run()
{
	this->window.show();
	this->label->show();

	int res = this->app.exec();
	this->algo->forceQuit();
	return res;
}

void MainInterface::create()
{
	window.setWindowTitle("Spherical Robot for Child Care");
	window.setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, window.size(),
		QRect(QPoint(10, 50), QPoint(window.width() + 10, window.height() + 50))));
	mainLayout = new QVBoxLayout();
	hboxLayout = new QHBoxLayout(&window);

	
	textResolution = new QLabel("Resolution of the camera : ");
	textDistanceArea = new QLabel("Minimal distance between two pixels belonging to the same area : ");
	textNbRobot = new QLabel("Maximal amount of robots that can appear on the camera pictures : ");
	textDisplayOpt = new QLabel("Display options : ");
	textHeight = new QLabel("Height distance between the camera and the ground (in centimeters) : ");
	textfreqLED1 = new QLabel("Blinking frequency of the LED1 (in milliseconds) : ");
	textfreqLED2 = new QLabel("Blinking frequency of the LED2 (in milliseconds) : ");
	label = new ClickableLabel();

	checkPosition = new QCheckBox("Display the position of the robot : ");
	checkOrientation = new QCheckBox("Display the orientation of the robot : ");
	checkIdentification = new QCheckBox("Display the identification of each region that can be recognized as a LED of the robot : ");
	checkKalman = new QCheckBox("Display the estimated position fo the robot (Kalman) : ");
	buttonCommand = new QCheckBox("Send command to the robot");

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

	comboBox = new QComboBox();

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

	// height of the camera
	mainLayout->addWidget(textHeight);
	mainLayout->addWidget(spinBoxHeight);
	mainLayout->addWidget(sliderHeight);

	QObject::connect(label, &ClickableLabel::clicked, std::bind(&Algo::setDesiredPoint, std::ref(this->algo), std::placeholders::_1, std::placeholders::_2));
	QObject::connect(spinBoxDist, SIGNAL(valueChanged(int)), sliderDist, SLOT(setValue(int)));
	QObject::connect(sliderDist, SIGNAL(valueChanged(int)), spinBoxDist, SLOT(setValue(int)));
	QObject::connect(sliderDist, &QSlider::valueChanged, std::bind(&Algo::setDistanceAreaLight, std::ref(this->algo), std::placeholders::_1));

	QObject::connect(spinBoxFreqLED1, SIGNAL(valueChanged(int)), sliderDist, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED1, SIGNAL(valueChanged(int)), spinBoxDist, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED1, &QSlider::valueChanged, std::bind(&Algo::setFreqLED1, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(spinBoxFreqLED2, SIGNAL(valueChanged(int)), sliderDist, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED2, SIGNAL(valueChanged(int)), spinBoxDist, SLOT(setValue(int)));
	QObject::connect(sliderFreqLED2, &QSlider::valueChanged, std::bind(&Algo::setFreqLED2, std::ref(this->algo), std::placeholders::_1));

	QObject::connect(spinBoxRobot, SIGNAL(valueChanged(int)), sliderRobot, SLOT(setValue(int)));
	QObject::connect(sliderRobot, SIGNAL(valueChanged(int)), spinBoxRobot, SLOT(setValue(int)));
	QObject::connect(sliderRobot, &QSlider::valueChanged, std::bind(&Algo::setNbRobot, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(spinBoxHeight, SIGNAL(valueChanged(int)), sliderHeight, SLOT(setValue(int)));
	QObject::connect(sliderHeight, SIGNAL(valueChanged(int)), spinBoxHeight, SLOT(setValue(int)));
	QObject::connect(sliderHeight, &QSlider::valueChanged, std::bind(&Algo::setHeight, std::ref(this->algo), std::placeholders::_1));

	void (QComboBox::*indexChangedSignal)(int) = &QComboBox::currentIndexChanged;
	QObject::connect(comboBox, indexChangedSignal, std::bind(&MainInterface::translateComboBox, this, std::placeholders::_1));
	QObject::connect(checkPosition, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayPosition, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkOrientation, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayOrientation, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkIdentification, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayIdentification, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkKalman, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayKalman, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(buttonCommand, &QCheckBox::stateChanged, std::bind(&Algo::sendCommand, std::ref(this->algo), std::placeholders::_1));
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
