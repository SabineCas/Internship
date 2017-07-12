/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : ClickableLabel
*/

#pragma once
#include <QLabel>
#include <Qt3DInput>

///! The ClickableLabel class inherits the QLabel class from the Qt library. This class is design to trigger the handle function when a left click on the label is done.
class ClickableLabel : public QLabel {
	Q_OBJECT

public:
	///! Constructor by default
	explicit ClickableLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());

	///! Destructor
	~ClickableLabel();

signals:
	///! Function signal of the ClickableLabel class that will be used to trigger the callback function linked to the instance
	/// of the class.
	/*!
	\param x The position of the clicked point on the X axis
	\param y The position of the clicked point on the Y axis
	\return void
	*/
	void clicked(int x, int y);

protected:
	///! Function called when a mouse event has occured. In the case that a left click has occured on the ClickableLabel
	/// instance, the function "clicked" will be call to trigger the callbakc function linked to this instance. 
	/*!
	\param event Mouse event that occured
	\return void
	*/
	void mousePressEvent(QMouseEvent* event);
};