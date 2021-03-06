/*
 * This file is part of the Motion_Detection_And_Tracking (MDT) project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */


#include "MDT.h"

MDT::MDT(){
	pMOG2 = createBackgroundSubtractorMOG2();
	dx = 0.001;
	dy = 0.001;
	loop = 0;
}

void MDT::extract_background(){
    pMOG2->apply(in, out);
    out2 = out.clone();
}

void MDT::get_countours(){
	vector<Point> game;	
	
	findContours(out, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );

	for( int i = 0; i < contours.size(); i++ ){
		approxPolyDP( Mat(contours[i]), contours_poly[i], 0, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	}

	for( int i = 0; i< contours.size(); i++ ){
		if(blobFilter(boundRect[i])){
			old_measurement = act_measurement;
			//rawCross(act_measurement, Scalar(255, 255, 255));
			act_measurement = Point(boundRect[i].x + (boundRect[i].width/2), boundRect[i].y  + (boundRect[i].height/2));
		}
	}

	labels.push_back(game);
	contours.clear();
	hierarchy.clear();
}

void MDT::bisects_gray(){
	/*for(int i = 0 ; i < in.cols ; i++){
		for(int j = 0 ; j < in.rows ; j++){
			Vec3b p = in.at<Vec3b>(j,i);
			cout << (int)p[0] << ", " << (int)p[1] << ", " << (int)p[2] << endl;
		}
	}*/
}

bool MDT::blobFilter(Rect rect){
	float proportion = (float)rect.width/rect.height;

	if((rect.width * rect.height) > AREA_MIN && (rect.width * rect.height) < AREA_MAX && proportion > PROPORTION_MIN && proportion < PROPORTION_MAX)
		return true;
	else
		return false;
}

void MDT::applyKF(){
    system("clear");

	measurement_X.at<float>(0) = act_measurement.x;
	measurement_Y.at<float>(0) = act_measurement.y;

	prediction_X = KFX.predict();
	prediction_Y = KFY.predict();

    cout << "Prediction Point(" << prediction_X.at<float>(0) << ", " << prediction_Y.at<float>(0) << ")" << endl;
    cout << "Measurement Point(" << measurement_X.at<float>(0) << ", " << measurement_Y.at<float>(0) << ")" << endl;

	//drawCross(Point(state_x, state_y), Scalar(255, 255, 255));
	drawCross(Point(measurement_X.at<float>(0), measurement_Y.at<float>(0)), Scalar(255, 255, 255));
	drawCross(Point(prediction_X.at<float>(0), prediction_Y.at<float>(0)), Scalar(0, 255, 0));

	if(theRNG().uniform(0,5) != 0){
    	KFX.correct(measurement_X);
    	KFY.correct(measurement_Y);
	}

    //state_X = KFX.transitionMatrix*state_X + ((act_measurement.x - old_measurement.x) *0.01);
	//state_Y = KFY.transitionMatrix*state_Y + ((act_measurement.y - old_measurement.y) *0.01);

    cout << endl;
}

void MDT::simKF(){
    init_sim_KF();
    while(true){
    	double state_x = state_X.at<float>(0);
		double state_y = state_Y.at<float>(0);

	    system("clear");
	    
	    cout << "state" << endl;
	    cout << state_X << endl;

		prediction_X = KFX.predict();
		prediction_Y = KFY.predict();

	    cout << "prediction" << endl;
	    cout << prediction_X << endl;

		double predict_x = prediction_X.at<float>(0);
		double predict_y = prediction_Y.at<float>(0);

		//measurement_X.at<float>(0) = 5;
		randn(measurement_X, Scalar::all(0), Scalar::all(KFX.measurementNoiseCov.at<float>(0)));
		randn(measurement_Y, Scalar::all(0), Scalar::all(KFY.measurementNoiseCov.at<float>(0)));

	    cout << "measurement" << endl;
	    cout << measurement_X << endl;

		measurement_X += KFX.measurementMatrix*state_X;
		measurement_Y += KFY.measurementMatrix*state_Y;

	    //cout << "measurementMatrix" << endl;
	    //cout << KFX.measurementMatrix << endl;

	    cout << "measurement" << endl;
	    cout << measurement_X << endl;

		double meas_x = measurement_X.at<float>(0);
		double meas_y = measurement_Y.at<float>(0);

		in = Scalar::all(0);	//simulation

		drawCross(Point(state_x + 250, state_y + 250), Scalar(255, 255, 255));
		drawCross(Point(meas_x + 250, meas_y + 250), Scalar(0, 0, 255));
		drawCross(Point(predict_x + 250, predict_y + 250), Scalar(0, 255, 0));

		if(theRNG().uniform(0,4) != 0){
	    	KFX.correct(measurement_X);
	    	KFY.correct(measurement_Y);
		}

		randn(processNoise_X, Scalar(0), Scalar::all(sqrt(KFX.processNoiseCov.at<float>(0, 0))));
		randn(processNoise_Y, Scalar(0), Scalar::all(sqrt(KFY.processNoiseCov.at<float>(0, 0))));

	    cout << "processNoise" << endl;
	    cout << processNoise_X << endl;

		state_X = KFX.transitionMatrix*state_X + processNoise_X + dx;
		state_Y = KFY.transitionMatrix*state_Y + processNoise_Y + dy;
	    
	    cout << endl;
		imshow("Kalman", in);

        usleep(10000);
        code = (char)waitKey(10);

        if( code > 0 )
            break;
    }
}

void MDT::init_sim_KF(){
	in = Mat(500, 500, CV_8UC3);

    KFX = KalmanFilter(2, 1, 0);
    KFY = KalmanFilter(2, 1, 0);

    state_X = Mat(2, 1, CV_32F);
    state_Y = Mat(2, 1, CV_32F);

    processNoise_X = Mat(2, 1, CV_32F);
    processNoise_Y = Mat(2, 1, CV_32F);

    measurement_X = Mat::zeros(1, 1, CV_32F);
    measurement_Y = Mat::zeros(1, 1, CV_32F);

    code = (char)-1;

    randn(state_X, Scalar::all(0), Scalar::all(0.2));
	randn(state_Y, Scalar::all(0), Scalar::all(0.2));

    cout << "first state" << endl;
    cout << state_X << endl;

	KFX.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
	KFY.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

	setIdentity(KFX.measurementMatrix);
    setIdentity(KFX.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KFX.measurementNoiseCov, Scalar::all(/*1e-1*/2.0));
    setIdentity(KFX.errorCovPost, Scalar::all(1.0));

    setIdentity(KFY.measurementMatrix);
    setIdentity(KFY.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KFY.measurementNoiseCov, Scalar::all(/*1e-1*/2.0));
    setIdentity(KFY.errorCovPost, Scalar::all(1.0));

    randn(KFX.statePost, Scalar::all(0), Scalar::all(0.1));
    randn(KFY.statePost, Scalar::all(0), Scalar::all(0.1));
}

void MDT::init_KF(Point pos){
    KFX = KalmanFilter(2, 1, 0);
    KFY = KalmanFilter(2, 1, 0);

    state_X = Mat(2, 1, CV_32F);
    state_Y = Mat(2, 1, CV_32F);

    processNoise_X = Mat(2, 1, CV_32F);
    processNoise_Y = Mat(2, 1, CV_32F);

    measurement_X = Mat::zeros(1, 1, CV_32F);
    measurement_Y = Mat::zeros(1, 1, CV_32F);

    code = (char)-1;

    //randn(state_X, Scalar::all(0), Scalar::all(0.2));
	//randn(state_Y, Scalar::all(0), Scalar::all(0.2));

    cout << "first state" << endl;
    cout << state_X << endl;

	KFX.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
	KFY.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

	setIdentity(KFX.measurementMatrix);
    setIdentity(KFX.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KFX.measurementNoiseCov, Scalar::all(1e-4));
    setIdentity(KFX.errorCovPost, Scalar::all(1.0));

    setIdentity(KFY.measurementMatrix);
    setIdentity(KFY.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KFY.measurementNoiseCov, Scalar::all(1e-4));
    setIdentity(KFY.errorCovPost, Scalar::all(1.0));

    randn(KFX.statePost, Scalar::all(0), Scalar::all(0.1));
    randn(KFY.statePost, Scalar::all(0), Scalar::all(0.1));
    //cout << KFY.statePost << endl;

    KFX.statePost.at<float>(0) = pos.x;
    KFY.statePost.at<float>(0) = pos.y;
}

void MDT::detect_and_track(){
	//capture = VideoCapture(0);
	//capture = VideoCapture("database/students/video.avi");
	//capture = VideoCapture("database/vss/video.mp4");
	capture = VideoCapture("database/ball/straight_1.mp4");
	//initKalman();

	while(true){
		//if(loop % 5 == 0){
			if(!capture.read(in) || key == 27) {
		        cerr << "Unable to read next frame." << endl;
		        cerr << "Exiting..." << endl;
		        exit(EXIT_FAILURE);
		    }
	    //}

		inRange(in, Scalar(0, 0, 80), Scalar(60, 255, 255), out);

	    //extract_background();
	    
	    medianBlur(out, out, 3);
	    
	    bisects_gray();
	    
	    get_countours();

	    if(act_measurement.x != 0){
			if(loop < 1)
				init_KF(act_measurement);
			
			/*if(loop > 30 && loop < 40){
				act_measurement = Point(-1, -1);
			}*/
			applyKF();
			loop++;
		}
	   

	    imshow("in", in);

	    key = waitKey(100);
	}
}

double MDT::calcX(double x){
	x += 0;
	return x;
}

double MDT::calcY(double y){
	y += 0;
	return y;
}

void MDT::drawCross(Point center, Scalar color){
	line(in, Point( center.x - 5, center.y - 5), Point( center.x + 5, center.y + 5 ), color, 2, LINE_AA, 0);
	line(in, Point( center.x + 5, center.y - 5), Point( center.x - 5, center.y + 5 ), color, 2, LINE_AA, 0);
}