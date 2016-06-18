#ifndef _MDT_
#define _MDT_

#include "iostream"
#include "sstream"

#include "opencv.h"

using namespace std;

class MDT{
protected:
	Mat in, out;
	Ptr<BackgroundSubtractor> pMOG2;
	VideoCapture capture;
	char key;

	void extract_background();

public:
	MDT();

	void detect_and_track();
};

#endif // _BACKGROUND_SUBTRACT_