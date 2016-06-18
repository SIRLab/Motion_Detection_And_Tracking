/*
 * This file is part of the Motion_Detection_And_Tracking (MDT) project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

/*#include "MDT.h"

int main(){
	MDT mdt;
	mdt.detect_and_track();
    //mdt.simKF();
}*/

#include "opencv.h"
#include "vector"
#include "iostream"
 
using namespace std;
 
// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
// <<<<< Color to be tracked
int main() { 
    VideoCapture cap;  
    unsigned char key; 
    // Camera frame    
    cv::Mat frame;      // >>>> Kalman Filter
   int stateSize = 6;
   int measSize = 4;
   int contrSize = 0;
 
   unsigned int type = CV_32F;
   cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
 
   cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
   cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
   //cv::Mat procNoise(stateSize, 1, type)
   // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
 
   // Transition State Matrix A
   // Note: set dT at each processing step!
   // [ 1 0 dT 0  0 0 ]
   // [ 0 1 0  dT 0 0 ]
   // [ 0 0 1  0  0 0 ]
   // [ 0 0 0  1  0 0 ]
   // [ 0 0 0  0  1 0 ]
   // [ 0 0 0  0  0 1 ]
   cv::setIdentity(kf.transitionMatrix);
 
   // Measure Matrix H
   // [ 1 0 0 0 0 0 ]
   // [ 0 1 0 0 0 0 ]
   // [ 0 0 0 0 1 0 ]
   // [ 0 0 0 0 0 1 ]
   kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
   kf.measurementMatrix.at<float>(0) = 1.0f;
   kf.measurementMatrix.at<float>(7) = 1.0f;
   kf.measurementMatrix.at<float>(16) = 1.0f;
   kf.measurementMatrix.at<float>(23) = 1.0f;
 
   // Process Noise Covariance Matrix Q
   // [ Ex 0  0    0 0    0 ]
   // [ 0  Ey 0    0 0    0 ]
   // [ 0  0  Ev_x 0 0    0 ]
   // [ 0  0  0    1 Ev_y 0 ]
   // [ 0  0  0    0 1    Ew ]
   // [ 0  0  0    0 0    Eh ]
   //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
   kf.processNoiseCov.at<float>(7) = 1e-2;
   kf.processNoiseCov.at<float>(14) = 2.0f;
   kf.processNoiseCov.at<float>(21) = 1.0f;
   kf.processNoiseCov.at<float>(28) = 1e-2;
   kf.processNoiseCov.at<float>(35) = 1e-2;
 
    // Measures Noise Covariance Matrix R
   cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
   // <<<< Kalman Filter      // Camera Index    int idx = 0;      // Camera Capture    cv::VideoCapture cap;      // >>>>> Camera Settings
   cap = VideoCapture("database/ball/straight_1.mp4");

    if(!cap.read(frame) || key == 27) {
        cerr << "Unable to read next frame." << endl;
        cerr << "Exiting..." << endl;
        exit(EXIT_FAILURE);
    }
 
   //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
   //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
   // <<<<< Camera Settings
 
   cout << "\nHit 'q' to exit...\n";      char ch = 0;      double ticks = 0;    bool found = false;      int notFoundCount = 0;            // >>>>> Main loop  
   while (ch != 'q' || ch != 'Q')
   {
      double precTick = ticks;
      ticks = (double) cv::getTickCount();
 
      double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
 
      // Frame acquisition
      cap >> frame;
 
      cv::Mat res;
      frame.copyTo( res );
 
      if (found)
      {
         // >>>> Matrix A
         kf.transitionMatrix.at<float>(2) = dT;
         kf.transitionMatrix.at<float>(9) = dT;
         // <<<< Matrix A
 
         cout << "dT:" << endl << dT << endl;
 
         state = kf.predict();
         cout << "State post:" << endl << state << endl;           
        cv::Rect predRect;         
         predRect.width = state.at<float>(4);   
                predRect.height = state.at<float>(5);  
                        predRect.x = state.at<float>(0) - predRect.width / 2;  
                                predRect.y = state.at<float>(1) - predRect.height / 2;  
                                          cv::Point center;          center.x = state.at<float>(0);  
                                                  center.y = state.at<float>(1);         
                                                   cv::circle(res, center, 2, CV_RGB(255,0,0), -1);  
                                                             cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);   
      }         // >>>>> Noise smoothing
      cv::Mat blur;
      cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
      // <<<<< Noise smoothing         // >>>>> HSV conversion
      cv::Mat frmHsv;
      cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
      // <<<<< HSV conversion         // >>>>> Color Thresholding
      // Note: change parameters for different colors
      cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
      cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
            cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
      // <<<<< Color Thresholding         // >>>>> Improving the result
      cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
      cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
      // <<<<< Improving the result         // Thresholding viewing       cv::imshow("Threshold", rangeRes);         // >>>>> Contours detection
      vector contours;
      cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
            CV_CHAIN_APPROX_NONE);
      // <<<<< Contours detection         // >>>>> Filtering
      vector balls;
      vector ballsBox;
      for (size_t i = 0; i < contours.size(); i++)       {          cv::Rect bBox;          bBox = cv::boundingRect(contours[i]);            float ratio = (float) bBox.width / (float) bBox.height;          if (ratio > 1.0f)
            ratio = 1.0f / ratio;
 
         // Searching for a bBox almost square
         if (ratio > 0.75 && bBox.area() >= 400)
         {
            balls.push_back(contours[i]);
            ballsBox.push_back(bBox);
         }
      }
      // <<<<< Filtering
 
      cout << "Balls found:" << ballsBox.size() << endl;         // >>>>> Detection result
      for (size_t i = 0; i < balls.size(); i++)
      {
         cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
         cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);
 
         cv::Point center;
         center.x = ballsBox[i].x + ballsBox[i].width / 2;
         center.y = ballsBox[i].y + ballsBox[i].height / 2;
         cv::circle(res, center, 2, CV_RGB(20,150,20), -1);
 
         stringstream sstr;
         sstr << "(" << center.x << "," << center.y << ")";
         cv::putText(res, sstr.str(),
               cv::Point(center.x + 3, center.y - 3),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
      }
      // <<<<< Detection result         // >>>>> Kalman Update
      if (balls.size() == 0)
      {
         notFoundCount++;
         cout << "notFoundCount:" << notFoundCount << endl;          if( notFoundCount >= 10 )
         {
            found = false;
         }
         else
            kf.statePost = state;
      }
      else
      {
         notFoundCount = 0;
 
         meas.at(0) = ballsBox[0].x + ballsBox[0].width / 2;
         meas.at(1) = ballsBox[0].y + ballsBox[0].height / 2;
         meas.at(2) = (float)ballsBox[0].width;
         meas.at(3) = (float)ballsBox[0].height;
 
         if (!found) // First detection!
         {
            // >>>> Initialization
            kf.errorCovPre.at(0) = 1; // px
            kf.errorCovPre.at(7) = 1; // px
            kf.errorCovPre.at(14) = 1;
            kf.errorCovPre.at(21) = 1;
            kf.errorCovPre.at(28) = 1; // px
            kf.errorCovPre.at(35) = 1; // px
 
            state.at(0) = meas.at(0);
            state.at(1) = meas.at(1);
            state.at(2) = 0;
            state.at(3) = 0;
            state.at(4) = meas.at(2);
            state.at(5) = meas.at(3);
            // <<<< Initialization
 
            found = true;
         }  
         else
            kf.correct(meas); // Kalman Correction
 
         cout << "Measure matrix:" << endl << meas << endl;
      }
      // <<<<< Kalman Update
 
      // Final result
      cv::imshow("Risultato finale", res);
 
      // User key
      ch = cv::waitKey(10);
   }
   // <<<<< Main loop
 
   return EXIT_SUCCESS;
}
