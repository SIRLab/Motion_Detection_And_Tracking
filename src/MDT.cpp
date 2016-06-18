#include "MDT.h"

MDT::MDT(){
	pMOG2 = createBackgroundSubtractorMOG2();
}

void MDT::extract_background(){
    pMOG2->apply(in, out);
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

	    imshow("in", in);
	    imshow("out", out);

	    key = waitKey(10);
	}
}