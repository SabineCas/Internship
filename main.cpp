/**
* Project : Detection and navigation of a spherical robot
* Author : Cassat Sabine
* Mail : sabinecassat@gmail.com
* Module : Main function
*/

#include "interface2.h"
#include "algo.h"

#include <windows.h>
#include <iostream>

int main(int argc, char** argv) {
	// Interface with Qt librairie only
	MainInterface i(argc, argv);
	Algo algo;
	algo.setInterface(&i);
	i.setAlgo(&algo);
	algo.start();
	return (i.run());
}