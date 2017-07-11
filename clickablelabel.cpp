/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : ClickableLabel
*/

#include "clickablelabel.h"
#include "clickablelabel.moc"

ClickableLabel::ClickableLabel(QWidget* parent, Qt::WindowFlags f)
	: QLabel(parent) {

}

ClickableLabel::~ClickableLabel() {}

void ClickableLabel::mousePressEvent(QMouseEvent * event) {
	// If the right button of the mouse is push, then we send a signal to the handle function
	if (event->button() == Qt::LeftButton) {
		emit clicked(event->pos().x(), event->pos().y());
	}
	// Confirmation that we received the QMouseEvent
	event->accept();
}