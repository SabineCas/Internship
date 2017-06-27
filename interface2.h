#pragma once

#include <memory>
#include <QApplication>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include <QComboBox>
#include <QCheckBox>
#include "clickablelabel.h"

class Algo;

class MainInterface {
public:
	//! Constructor that the input of the main function as parameters.
	/*!
	\param argc Argc parameter of the main function
	\param argv Argv parameter of the main function
	\return void
	*/
	MainInterface(int argc, char * argv[]);

	//! Initialize the pointer to the algorithm that will update the graphical interface or be updated by the graphical
	//! interface.
	/*!
	\param a The pointer algorithm instance that will update the graphical interface or be updated by the graphical interface
	\return void
	*/
	void setAlgo(Algo * a);

	//! Function that will start the interface, but be careful this is a blocking function. You cannot taking back control 
	//! of the algorithm.
	/*!
	\param void
	\return int Return 0 if everything went well, otherwise there was an issue to the execution of the interface
	*/
	int run();

	//! Function that will stop the interface and close the multithread properly.
	/*!
	\param void
	\return void
	*/
	void end();

	//! Set the value of the QSpinBox spinbox attribut that will be used to set the maximal distance between two pixels
	//! that will belong to the same area. This function will be called to update the interface from the instance of Algo.
	/*!
	\param int The maximal distance value
	\return void
	*/
	void setDistance(int);

	//! Set the data of the QImage contained inside the QLabel label attribut that will be used to display the 
	// image of the camera at each loop of the algorithm. 
	/*!
	\param i The QImage that will be load inside the QLabel label attribut 
	\return void
	*/
	void SetImage(const QImage& i);

	//! Translate the signification of the chosen index of the QComboBox combox attribut that will be used to set
	//! the resolution of the camera.
	/*!
	\param v The index of the selected item of the QCombobox combobox attribut
	\return void
	*/
	void translateComboBox(int v);

private:
	//! Create the QObjects and connect them to the interface by positionning them and create a signal that will act
	//! like a callback function.
	/*!
	\param void
	\return void
	*/
	void create();

	Algo * algo;
	QApplication app;
	QWidget window;
	QVBoxLayout* mainLayout;
	QHBoxLayout* hboxLayout;
	ClickableLabel * label;

	QLabel * textResolution;
	QLabel * textDisplayOpt;
	QLabel * textDistanceArea;
	QLabel * textNbRobot;
	QLabel * textHeight;
	QLabel * textfreqLED1;
	QLabel * textfreqLED2;

	QCheckBox * checkPosition;
	QCheckBox * checkOrientation;
	QCheckBox * checkIdentification;
	QCheckBox * checkKalman;
	QCheckBox * buttonCommand;

	QSpinBox * spinBoxDist;
	QSlider * sliderDist;
	QSpinBox * spinBoxRobot;
	QSlider * sliderRobot;
	QSpinBox * spinBoxHeight;
	QSlider * sliderHeight;
	QSpinBox * spinBoxFreqLED1;
	QSlider * sliderFreqLED1;
	QSpinBox * spinBoxFreqLED2;
	QSlider * sliderFreqLED2;

	QComboBox * comboBox;
};