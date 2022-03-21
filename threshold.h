//umbrales
#include<iostream>
#include<opencv2/calib3d.hpp>
#include<opencv2/core/types.hpp>
#include<opencv2/core/persistence.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core/types_c.h>
#include <utility>
#include "functions.h"

using namespace std;
using namespace cv;

class thresholds{
  public:
    thresholds(string path):PATH(path){}

    void get_combined_gradients()
    {


        Mat  ImgThreshold = imread(PATH+"test3.jpg");

        //Mat temp = ImgThreshold.clone();
        Mat R_channel,sobelx,sobely;
        //cvtColor(ImgThreshold,temp,COLOR_BGR2GRAY);

        funciones fun;

        double rows_ = ImgThreshold.rows;
        double cols_ = ImgThreshold.cols;


        if(ImgThreshold.empty())
        {
            cout<< "image file"
                << "no founf"<<endl;
            cin.get();
            return;
        }

        ImgThreshold = fun.ResizeImage(ImgThreshold,0.5);
        //ImgThreshold.copyTo(R_channel);

        int x = 7; // x coordinate of the top-left corner
        int y = 227; // y coordinate of the top-left corner

        cv::Rect roi_rect;
        roi_rect.x = x;
        roi_rect.y = y;
        roi_rect.width = ImgThreshold.cols-12;
        roi_rect.height = 120;

        R_channel = ImgThreshold(roi_rect);

        sobelx = abs_sobel_thresh(R_channel,'x');


        imshow("image thresholds",ImgThreshold);
        imshow("image temp",R_channel);
        waitKey(0);
    }

    Mat abs_sobel_thresh(Mat img,char orient='x',pair<double,double>thresh = std::make_pair(20, 100))
    {
        Mat abs_sobel;

        if(orient == 'x'){
            Sobel(img,abs_sobel,CV_64F,1,0,5);
            abs(abs_sobel);
        }
        if(orient == 'y'){
            Sobel(img,abs_sobel,CV_64F,0,1,5);
            abs(abs_sobel);
        }

        imshow("sobel",abs_sobel);

        return abs_sobel;
    }

private:
    pair<double,double> th_sobelx = std::make_pair(35, 100);
    pair<double,double> th_sobely = std::make_pair(30, 255);
    pair<double,double> th_mag = std::make_pair(30, 255);
    pair<double,double> th_dir = std::make_pair(0.7, 1.3);
    // get my img
    string PATH;


};
