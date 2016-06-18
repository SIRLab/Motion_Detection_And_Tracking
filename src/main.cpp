/*#include "MDT.h"

int main(){
	MDT mdt;
	mdt.detect_and_track();
}*/

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "unistd.h"
#include "iostream"

#include <stdio.h>

using namespace std;
using namespace cv;

static inline Point calcPoint(Point2f center, double R, double angle){
    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}

static inline double calcX(double x){
	x += 20;
	return x;
}

static inline double calcY(double y){
	y += 20;
	return y;
}

// plot points
        #define drawCross( center, color, d )                                        \
            line( img, Point( center.x - d, center.y - d ),                          \
                         Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
            line( img, Point( center.x + d, center.y - d ),                          \
                         Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )

int main(){
	srand(time(NULL));
    int loop = 0;

    Mat img(500, 500, CV_8UC3);

    KalmanFilter KFX(2, 1, 0);
    KalmanFilter KFY(2, 1, 0);

    Mat state_X(2, 1, CV_32F);
    Mat state_Y(2, 1, CV_32F);

    Mat processNoise_X(2, 1, CV_32F);
    Mat processNoise_Y(2, 1, CV_32F);

    Mat measurement_X = Mat::zeros(1, 1, CV_32F);
    Mat measurement_Y = Mat::zeros(1, 1, CV_32F);

    char code = (char)-1;

    /*inicio de alguma coisa rsrsrs*/{
    	randn(state_X, Scalar::all(0), Scalar::all(0.2));
    	randn(state_Y, Scalar::all(0), Scalar::all(0.2));

        cout << "first state" << endl;
        cout << state_X << endl;

    	KFX.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
    	KFY.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

    	setIdentity(KFX.measurementMatrix);
        setIdentity(KFX.processNoiseCov, Scalar::all(1e-5));
        setIdentity(KFX.measurementNoiseCov, Scalar::all(/*1e-1*/5.0));
        setIdentity(KFX.errorCovPost, Scalar::all(1));

        setIdentity(KFY.measurementMatrix);
        setIdentity(KFY.processNoiseCov, Scalar::all(1e-5));
        setIdentity(KFY.measurementNoiseCov, Scalar::all(/*1e-1*/5.0));
        setIdentity(KFY.errorCovPost, Scalar::all(1));

        randn(KFX.statePost, Scalar::all(0), Scalar::all(0.1));
        randn(KFY.statePost, Scalar::all(0), Scalar::all(0.1));

        while(true){
        	Point2f center(img.cols*0.5f, img.rows*0.5f);
        	float R = img.cols/3.f;

        	double state_x = state_X.at<float>(0);
        	double state_y = state_Y.at<float>(0);

            if(loop % 3 == 0)
            system("clear");
            
            cout << "state" << endl;
            cout << state_X << endl;

        	double state_x_Pt = calcX(state_x);
        	double state_y_Pt = calcY(state_y);

        	Mat prediction_X = KFX.predict();
        	Mat prediction_Y = KFY.predict();

            cout << "prediction" << endl;
            cout << prediction_X << endl;

        	double predict_x = prediction_X.at<float>(0);
        	double predict_y = prediction_Y.at<float>(0);

        	double predict_x_Pt = calcX(predict_x);
        	double predict_y_Pt = calcY(predict_y);

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

        	double meas_x_Pt = calcX(meas_x);
        	double meas_y_Pt = calcY(meas_y);

        	img = Scalar::all(0);

        	drawCross( Point(state_x_Pt, state_y_Pt), Scalar(255,255,255), 3 );
        	drawCross( Point(meas_x_Pt, meas_y_Pt), Scalar(0,0,255), 3 );
        	drawCross( Point(predict_x_Pt, predict_y_Pt), Scalar(0,255,0), 3 );

        	if(theRNG().uniform(0,4) != 0){
            	KFX.correct(measurement_X);
            	KFY.correct(measurement_Y);
        	}

        	randn( processNoise_X, Scalar(0), Scalar::all(sqrt(KFX.processNoiseCov.at<float>(0, 0))));
        	randn( processNoise_Y, Scalar(0), Scalar::all(sqrt(KFY.processNoiseCov.at<float>(0, 0))));

            cout << "processNoise" << endl;
            cout << processNoise_X << endl;

        	state_X = KFX.transitionMatrix*state_X + processNoise_X;
        	state_Y = KFY.transitionMatrix*state_Y + processNoise_Y;

            //cout << "transitionMatrix" << endl;
            //cout << KFX.transitionMatrix << endl;
            
            cout << endl;
        	imshow( "Kalman", img );
            if(loop < 100){
                loop++;
                usleep(200000);
            }else{
                usleep(2000000);
            }
	        code = (char)waitKey(10);

	        if( code > 0 )
	            break;
	    }

    }
}

int main_old(int, char**){
    Mat img(500, 500, CV_8UC3);
    KalmanFilter KF(2, 1, 0);
    Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
    Mat processNoise(2, 1, CV_32F);
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    char code = (char)-1;

 
    randn( state, Scalar::all(0), Scalar::all(0.1) );
    KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(1));

    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

    for(;;)
    {
        Point2f center(img.cols*0.5f, img.rows*0.5f);
        float R = img.cols/3.f;
        double stateAngle = state.at<float>(0);
        Point statePt = calcPoint(center, R, stateAngle);

        Mat prediction = KF.predict();
        double predictAngle = prediction.at<float>(0);
        Point predictPt = calcPoint(center, R, predictAngle);

        randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));

        // generate measurement
        measurement += KF.measurementMatrix*state;

        double measAngle = measurement.at<float>(0);
        Point measPt = calcPoint(center, R, measAngle);

        img = Scalar::all(0);
        drawCross( statePt, Scalar(255,255,255), 3 );
        drawCross( measPt, Scalar(0,0,255), 3 );
        drawCross( predictPt, Scalar(0,255,0), 3 );

        /*if(theRNG().uniform(0,4) != 0)
            KF.correct(measurement);*/

        randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
        state = KF.transitionMatrix*state + processNoise;

        imshow( "Kalman", img );
        code = (char)waitKey(100);

        if( code > 0 )
            break;
    }

    return 0;
}
