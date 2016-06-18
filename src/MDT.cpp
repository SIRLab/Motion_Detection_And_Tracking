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

void MDT::detect_and_track(){
	//capture = VideoCapture(0);
	capture = VideoCapture("database/students/video.avi");
	
	while(true){
		if(!capture.read(in) || key == 27) {
	        cerr << "Unable to read next frame." << endl;
	        cerr << "Exiting..." << endl;
	        exit(EXIT_FAILURE);
	    }

	    extract_background();
	    
	    imshow("MOG", out);
	    
	    medianBlur(out, out, 3);
	    
	    bisects_gray();
	    
	    get_countours();
	   

	    imshow("in", in);
	    imshow("out", out);

	    key = waitKey(100);
	}
}