/*
 * This file is part of the Motion_Detection_And_Tracking (MDT) project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

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

    Mat img;//(500, 500, CV_8UC3);

    KalmanFilter KFX;
    Mat state_X;
    Mat prediction_X;
    Mat processNoise_X;
    Mat measurement_X;

    KalmanFilter KFY;
    Mat state_Y;
    Mat prediction_Y;
    Mat processNoise_Y;
    Mat measurement_Y;

    float dx, dy;

	char key, code;
	int loop;

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

	void initKF();
	void applyKF();
};

#endif // _BACKGROUND_SUBTRACT_