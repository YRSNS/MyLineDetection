//class functions
#include<opencv2/calib3d.hpp>
#include<opencv2/core/types.hpp>
#include<opencv2/core/persistence.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core/types_c.h>
#include<iostream>
using namespace cv;
using namespace std;

class funciones{

 public:

    void ReadVideo()
    {
        VideoCapture cap;
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 400); // valueX = your wanted width
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 300); // valueY = your wanted heigth
        cap.open(2);
        cap.set(cv::CAP_PROP_FPS, 30);

        while(1){
           Mat frame;
           cap >> frame;
           if (frame.empty())
             break;
           imshow( "Frame", frame);
           char c=(char)waitKey(25);
           if(c==27)
             break;
         }
         cap.release();
         destroyAllWindows();
    }

    void ReadImage(string name,cv::Mat image){
        Mat  img,imgGray,roi_image;
        img = image.clone();
        //img = imread(PATH+"test2.jpg");
        //cvtColor(img,imgGray,COLOR_BGR2GRAY);

        if(img.empty())
        {
            cout<< "image file"
                << "no founf"<<endl;
            cin.get();
            return;
        }
        //int roi_width = img.rows * 0.5;
        //int roi_height = img.cols* 0.5;


        img = ResizeImage(img,0.5);

        int x = 7; // x coordinate of the top-left corner
        int y = 227; // y coordinate of the top-left corner

        cv::Rect roi_rect;
        roi_rect.x = x;
        roi_rect.y = y;
        roi_rect.width = img.cols-12;
        roi_rect.height = 120;

        roi_image = img(roi_rect);



        imshow(name,img);
        //imshow("roirect",roi_image);
        //imshow("image gray",imgGray);
        //waitKey(0);

    }

    Mat ResizeImage(Mat &image, float scale)
    {
        Mat resized;
        resize(image, resized, Size(image.cols*scale, image.rows*scale), INTER_LINEAR);

        return resized;
    }

};


