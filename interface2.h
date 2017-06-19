#pragma once

#include <memory>
#include <QApplication>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include <QComboBox>
#include <QCheckBox>

class Algo;

class MainInterface {
public:
	MainInterface(int argc, char * argv[]);
	// Just for initialize the pointer to the algorithm that will update the graphical interface
	void setAlgo(Algo *);

	// Functions that start and stop the interface
	// Warning : running is a blocking function
	int run();
	void end();

	// Fucntions called to update the interface from the instance of Algo
	void setValue(int);
	void SetImage(const QImage&);
	void translateComboBox(int);

private:
	// Create the QObject and connect them to the interface (positionning + create signals <=> callback)
	void create();

	Algo * algo;
	QApplication app;
	QWidget window;
	QVBoxLayout* mainLayout;
	QHBoxLayout* hboxLayout;
	QLabel* label;
	QLabel * textResolution;

	QCheckBox * checkPosition;
	QCheckBox * checkOrientation;
	QCheckBox * checkIdentification;
	QCheckBox * checkKalman;

	QSpinBox* spinBox;
	QSlider* slider;
	QComboBox * comboBox;
};