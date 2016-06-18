#include "MDT.h"

MDT::MDT(){
	pMOG2 = createBackgroundSubtractorMOG2();
}

void MDT::extract_background(){
    pMOG2->apply(in, out);
    out2 = out.clone();
}

void MDT::get_countours(){
	vector<Point> game;	
	
	imshow("Median", out);
	findContours(out, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );

	for( int i = 0; i < contours.size(); i++ ){
		approxPolyDP( Mat(contours[i]), contours_poly[i], 0, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	}

	for( int i = 0; i< contours.size(); i++ ){
		rectangle(in, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0,0,255));
		//if(itsALabel(boundRect[i])){
			//game.push_back(Point(boundRect[i].x + (boundRect[i].width/2), boundRect[i].y  + (boundRect[i].height/2)));
		//}
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

void MDT::applyKF(){
	srand(time(NULL));
    int loop = 0;

    img = Mat(500, 500, CV_8UC3);

    KFX = KalmanFilter(2, 1, 0);
    KFY = KalmanFilter(2, 1, 0);

    state_X = Mat(2, 1, CV_32F);
    state_Y = Mat(2, 1, CV_32F);

    processNoise_X = Mat(2, 1, CV_32F);
    processNoise_Y = Mat(2, 1, CV_32F);

    measurement_X = Mat::zeros(1, 1, CV_32F);
    measurement_Y = Mat::zeros(1, 1, CV_32F);

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
                usleep(100000);
            }else{
                usleep(2000000);
            }
	        code = (char)waitKey(10);

	        if( code > 0 )
	            break;
	    }

    }
}

void MDT::detect_and_track(){
	//capture = VideoCapture(0);
	//capture = VideoCapture("database/students/video.avi");
	capture = VideoCapture("database/vss/video.mp4");

	while(true){
		if(!capture.read(in) || key == 27) {
	        cerr << "Unable to read next frame." << endl;
	        cerr << "Exiting..." << endl;
	        exit(EXIT_FAILURE);
	    }

	    Mat rep, rep2;
	    Rect cut = Rect(Point(70, 20), Point(350, 310));

		rep = in(cut);
			
		resize(rep, rep2, Size(640, 480), 0, 0, 0);
		in = rep2.clone();

		inRange(in, Scalar(0, 0, 170), Scalar(200, 150, 255), out);
	    //extract_background();
	    
	    imshow("MOG", out);
	    
	    medianBlur(out, out, 3);
	    
	    bisects_gray();
	    
	    get_countours();
	   

	    imshow("in", in);
	    imshow("out", out);

	    key = waitKey(100);
	}
}

double MDT::calcX(double x){
	x += 20;
	return x;
}

double MDT::calcY(double y){
	y += 20;
	return y;
}

void MDT::drawCross(Point center, Scalar color, int d){
	line(img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0);
	line(img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0);
}