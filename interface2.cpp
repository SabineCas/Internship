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
	mainLayout = new QVBoxLayout();
	hboxLayout = new QHBoxLayout(&window);

	textResolution = new QLabel("Resolution of the camera : ");
	label = new QLabel("Image");
	checkPosition = new QCheckBox("Display the position of the robot : ");
	checkOrientation = new QCheckBox("Display the orientation of the robot : ");
	checkIdentification = new QCheckBox("Display the identification of each region that can be recognized as a LED of the robot : ");
	checkKalman = new QCheckBox("Display the estimated position fo the robot (Kalman) : ");
	spinBox = new QSpinBox;
	slider = new QSlider(Qt::Horizontal);
	comboBox = new QComboBox();

	hboxLayout->addWidget(label);
	hboxLayout->addLayout(mainLayout);

	// Setting the resolution of the camera
	mainLayout->addWidget(textResolution);
	comboBox->addItem("640 x 480");
	comboBox->addItem("1280 x 720");
	comboBox->addItem("1920 x 1080");
	mainLayout->addWidget(comboBox);

	// Setting the displayed information
	mainLayout->addWidget(checkPosition);
	mainLayout->addWidget(checkOrientation);
	mainLayout->addWidget(checkIdentification);
	mainLayout->addWidget(checkKalman);

	// Distance between two pixels to belong to the same area
	mainLayout->addWidget(spinBox);
	mainLayout->addWidget(slider);
	

	QObject::connect(spinBox, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
	QObject::connect(slider, SIGNAL(valueChanged(int)), spinBox, SLOT(setValue(int)));
	void (QComboBox::*indexChangedSignal)(int) = &QComboBox::currentIndexChanged;
	QObject::connect(comboBox, indexChangedSignal, std::bind(&MainInterface::translateComboBox, this, std::placeholders::_1));
	QObject::connect(checkPosition, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayPosition, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkOrientation, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayOrientation, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkIdentification, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayIdentification, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(checkKalman, &QCheckBox::stateChanged, std::bind(&Algo::setDisplayKalman, std::ref(this->algo), std::placeholders::_1));
	QObject::connect(slider, &QSlider::valueChanged, std::bind(&Algo::setDistanceAreaLight, std::ref(this->algo), std::placeholders::_1));
}

void MainInterface::end()
{
	this->app.exit();
}

void MainInterface::setValue(int i)
{
	this->spinBox->setValue(i);
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
