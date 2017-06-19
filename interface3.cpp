//#include "interface3.h"
//
//Interface3::Interface3()
//{
//	this->labelImage = new QLabel("Image");
//	QObject::connect(this, SIGNAL(valueChanged(QPixmap)), labelImage, SLOT(setLabel(QPixelmap)));
//	this->labelImage->show();
//}
//
//void Interface3::run()
//{
//	Camera cam(0);
//	cv::Mat image;
//
//	while (1) {
//		cam.getCap().read(image);
//		cv::cvtColor(image, image, CV_BGR2RGB);
//		QImage img((uchar*)image.data, image.cols, image.rows, image.step1(), QImage::Format_RGB32);
//		this->setLabel(img);
//	}
//}
//
//void Interface3::setLabel(QImage img)
//{
//	this->labelImage->setPixmap(QPixmap::fromImage(img));
//}


#include "interface3.h"
#include "interface3.moc"

using namespace std;
using namespace cv;

/// QMat


QMat::QMat(QWidget *parent) : QWidget(parent)
{
	_Label = new QLabel(this);
	setLabel();
}

QMat::QMat(Mat newMat, QWidget *parent) : QWidget(parent), _mat(newMat)
{
	_Label = new QLabel(this);
	setLabel();
}

QMat::QMat(Mat(*mat_function)(), int waitTime, QWidget *parent) : QWidget(parent), _function(mat_function)
{
	_timer = new QTimer(this);
	_timer->start(waitTime);
	_mat = _function();
	_Label = new QLabel(this);
	connect(_timer, SIGNAL(timeout()), this, SLOT(getFunctionMat()));
	setLabel();
}

void QMat::setWaitingTime(int newt)
{
	_timer->start(newt);
}

void QMat::getFunctionMat()
{
	_mat = _function();
	emit matChanged();
	setLabel();
}

void QMat::setMat(cv::Mat newMat)
{
	_mat = newMat;
	setLabel();
}

void QMat::setFunction(cv::Mat(*new_function)())
{
	_function = new_function;
}

void QMat::setLabel()
{
	if (_mat.cols*_mat.rows)
	{
		Mat rgb;
		QPixmap p;
		cv::cvtColor(_mat, rgb, (-2 * _mat.channels() + 10));
		p.convertFromImage(QImage(rgb.data, rgb.cols, rgb.rows, QImage::Format_RGB888));
		_Label->setPixmap(p);
		resize(_mat.cols, _mat.rows);
	}
}