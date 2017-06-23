#include "clickablelabel.h"
#include "clickablelabel.moc"

ClickableLabel::ClickableLabel(QWidget* parent, Qt::WindowFlags f)
	: QLabel(parent) {

}

ClickableLabel::~ClickableLabel() {}

void ClickableLabel::mousePressEvent(QMouseEvent * event) {
	if (event->button() == Qt::LeftButton) {
		emit clicked(event->pos().x(), event->pos().y());
	}
	event->accept();
}