/***********************************
 *  name: sanchez yucra yhon yerson
 *  info: CV 2022
 *  proy: self driving car
 ***********************************/

#include<opencv2/opencv.hpp>
#include<iostream>
#include <opencv2/imgproc.hpp>
#include "cam.h"
//#include "functions.h"
#include "laneDetection.h"

using namespace std;
using namespace cv;



std::string PATH = "/home/y3rsn/Dev/cpp/CV_2022/MyLineDetection/img/calb/";


camera cam(PATH,2);
funciones fun;


VideoCapture laneVideo;
float laneDistant = 0;

Mat videoFrame; // Video Frame.
Mat videoFrameUndistorted; // Video Frame (after calibration).
Mat videoFramePerspective; // Video Frame (after perspective transform).
Mat _videoFrameUndistorted;
Mat debugWindow(540, 1280, CV_8UC3, Scalar(0,0,0)); //The Debug window.
Size videoSize; // Input Variable Size.
Mat cameraMatrix, dist; //Calibration Matrix.
Mat perspectiveMatrix; //Homography Matrix.
String coordinatetext = "";
Point2f perspectiveSrc[] = {Point2f(565,470), Point2f(721,470), Point2f(277,698), Point2f(1142,698)};
Point2f perspectiveDst[] = {Point2f(300,0), Point2f(980,0), Point2f(300,720), Point2f(980,720)};

int main(int argc, char **argv)
{
    //Get the Perspective Matrix.
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);



    laneVideo.open("/home/y3rsn/Dev/cpp/CV_2022/MyLineDetection/video/road.mp4");
    videoSize = Size((int)laneVideo.get(CAP_PROP_FRAME_WIDTH),(int)laneVideo.get(CAP_PROP_FRAME_HEIGHT));


    //--------------Camera Calibration Start-----------------
    FileStorage fsRead;
    //--------------copy Intrinsic.xml in build
    fsRead.open("Intrinsic.xml", FileStorage::READ);
    Mat src = imread(PATH+"Cam2.jpg");
    Mat dst;

    if (fsRead.isOpened() == false)
    {
        cam.doCalibration(cameraMatrix, dist);
        FileStorage fs;
        fs.open("Intrinsic.xml", FileStorage::WRITE);
        fs << "CameraMatrix" << cameraMatrix;
        fs << "Dist" << dist;
        fs.release();
        fsRead.release();
        cout << "There is no existing intrinsic parameters XML file." << endl;
        cout << "Start calibraton......" << endl;
    }
    else
    {
        fsRead["CameraMatrix"] >> cameraMatrix;
        fsRead["Dist"] >> dist;
        fsRead.release();
    }
    undistort(src, dst, cameraMatrix, dist);

    //--------------Camera Calibration Finish-----------------

    //Display Video Image   -->  get firts frame
    laneVideo.set(CAP_PROP_POS_FRAMES, 0);
    laneVideo >> videoFrame;
    //namedWindow("Original Image", WINDOW_NORMAL);
    //imshow("Original Image",videoFrame);


    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();



    //Start Homography
    warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);
    //namedWindow("Undistorsion", WINDOW_NORMAL);
    //imshow("videoFramePerspective",videoFramePerspective);
    //fun.ReadImage("videoFrame",videoFramePerspective);

    //Applying lane detection algorithm
    laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
    //LaneAlgo.laneDetctAlgo();


    //Mat warpEdge; warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();
    //fun.ReadImage("canny",warpEdge);


    Mat finalResult;
    Mat mergeImage;
    Mat warpimage;
    Mat otherwarp;
    //= LaneAlgo.getFinalResult().clone();
    //imshow("resultado",finalResult);
    //fun.ReadImage("resultado",finalResult);



    //===========Finish Real Time Processing===========
    stringstream ss;
    //========== procesing using video ============
    while(!videoFrame.empty())
    {
        //Start Homography
        warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

        //Applying lane detection algorithm
        //if(videoFrameCount == 0)
        LaneAlgo.laneDetctAlgo();
        //finalResult = LaneAlgo.getFinalResult();
        finalResult = videoFrame;

        mergeImage = LaneAlgo.getMergeImage();

        warpimage = LaneAlgo.getMaskImage();
        otherwarp = LaneAlgo.getWarpMask();


        ss.str("");
        ss.clear();
        ss << "          Road  ";
        putText(finalResult, ss.str(), Point(50,50), 0, 2, Scalar(0, 0, 255), 2);


        imshow("Real Time Execution", fun.ResizeImage(finalResult,0.3));
        imshow("mergeimage",fun.ResizeImage(mergeImage,0.3));
        //imshow("warpimage",fun.ResizeImage(warpimage,0.3));
        //imshow("other warp",fun.ResizeImage(otherwarp,0.3));
        //imshow("mergeimage",fun.ResizeImage(mergeImage,0.3));

        //write the video
        laneVideo >> videoFrame;
        if(videoFrame.empty()) break;


        //Calibration
        undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
        _videoFrameUndistorted = videoFrameUndistorted.clone();
        LaneAlgo.setInputImage(_videoFrameUndistorted);

        if(waitKey(10) == 27) break;
    }



    waitKey(0);
    laneVideo.release();
    destroyAllWindows();
    return 0;

}
