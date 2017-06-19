#pragma once

#include <memory>
#include <QtGui\qimage.h>

class Algo;

class Interface {
public:
	Interface(int argc, char * argv[]);
	int run();
	void end();

	void setValue(int);
	void SetImage(const QImage&);

	void setAlgo(Algo *);

private:
	class Impl;
	std::shared_ptr<Impl> impl;
};