

#include <windows.h>
#include "interface2.h"

#include <QApplication>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include "algo.h"

class Interface::Impl {
public:

	void create();
	Impl(int argc, char *argv[]);
	void run();
	Algo * algo;

	// main app
	QApplication app;

	//window
	QWidget window;
	QVBoxLayout* mainLayout;
	QLabel* label;
	QSpinBox* spinBox;
	QSlider* slider;
};


Interface::Impl::Impl(int argc, char * argv[])
	: app(argc, argv)
{
}
void Interface::Impl::run()
{
	app.exec();
}
void Interface::Impl::create()
{

	mainLayout = new QVBoxLayout(&window);
	label = new QLabel("Image");
	spinBox = new QSpinBox;
	slider = new QSlider(Qt::Horizontal);

	mainLayout->addWidget(label);
	mainLayout->addWidget(spinBox);
	mainLayout->addWidget(slider);

	//QObject::connect(spinBox, SIGNAL(valueChanged(int)), label, SLOT(setNum(int)));
	QObject::connect(spinBox, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
	//QObject::connect(slider, SIGNAL(valueChanged(int)), label, SLOT(setNum(int)));
	QObject::connect(slider, SIGNAL(valueChanged(int)), spinBox, SLOT(setValue(int)));

	// both works
	//QObject::connect(slider, &QSlider::valueChanged, [=](int value) {this->algo->SetValue(value); });
	QObject::connect(slider, &QSlider::valueChanged, std::bind(&Algo::setValue, std::ref(this->algo), std::placeholders::_1));

	QImage image(1280, 720, QImage::Format_RGB32);
	label->resize(QSize(1280, 720));
	label->setPixmap(QPixmap::fromImage(image));
}

Interface::Interface(int argc, char * argv[])
{
	impl.reset(new Impl(argc, argv));
	impl->create();
}

int Interface::run()
{
	impl->window.show();
	impl->label->show();

	int res = impl->app.exec();
	impl->algo->forceQuit();
	return res;
}

void Interface::end()
{
	impl->app.exit();
}

void Interface::setValue(int i)
{
	impl->spinBox->setValue(i);
}

void Interface::SetImage(const QImage & image)
{
	QPixmap p;
	p.convertFromImage(image);
	impl->label->setPixmap(p);
}

void Interface::setAlgo(Algo *a)
{
	impl->algo = a;
}
