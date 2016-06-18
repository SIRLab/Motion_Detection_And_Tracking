#ifndef _MDT_
#define _MDT_

#include <stdio.h>

#include "iostream"
#include "sstream"

#include "unistd.h"

#include "opencv.h"

using namespace std;

class MDT{
protected:
	Mat in, out, out2;
	Ptr<BackgroundSubtractor> pMOG2;
	VideoCapture capture;
	
	vector<vector<Point> > labels;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	int loop;

    Mat img;//(500, 500, CV_8UC3);

    KalmanFilter KFX;//(2, 1, 0);
    KalmanFilter KFY;//(2, 1, 0);

    Mat state_X;//(2, 1, CV_32F);
    Mat state_Y;//(2, 1, CV_32F);

    Mat processNoise_X;//(2, 1, CV_32F);
    Mat processNoise_Y;//(2, 1, CV_32F);

    Mat measurement_X;// = Mat::zeros(1, 1, CV_32F);
    Mat measurement_Y;// = Mat::zeros(1, 1, CV_32F);

	char key;

	void extract_background();
	void get_countours();
	void bisects_gray();

	double calcX(double x);
	double calcY(double y);
	void drawCross(Point center, Scalar color, int size);
	

	//void detect_human();

public:
	MDT();

	void detect_and_track();
	void applyKF();
};

#endif // _BACKGROUND_SUBTRACT_