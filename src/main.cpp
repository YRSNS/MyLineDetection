/***********************************
 *  name: sanchez yucra yhon yerson
 *  info: CV 2022
 *  proy: self driving car
 ***********************************/

#include<opencv2/opencv.hpp>
#include<iostream>
#include <opencv2/imgproc.hpp>
#include "cam.h"
#include "functions.h"
#include "laneDetection.h"

using namespace std;
using namespace cv;



std::string PATH = "/home/yhon/Dev/MyLineDetection/img/calb/";

camera cam(PATH,2);
funciones fun;


VideoCapture laneVideo;
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



    laneVideo.open("/home/yhon/Dev/MyLineDetection/video/road.mp4");
    videoSize = Size((int)laneVideo.get(CAP_PROP_FRAME_WIDTH),(int)laneVideo.get(CAP_PROP_FRAME_HEIGHT));


    //--------------Camera Calibration Start-----------------
    FileStorage fsRead;
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
    imshow("videoFramePerspective",videoFramePerspective);

    //Applying lane detection algorithm
    laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
    LaneAlgo.laneDetctAlgo();


    Mat warpEdge; warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();
    fun.ReadImage("canny",warpEdge);


    Mat imageRedChannel; imageRedChannel = LaneAlgo.getRedChannel().clone();

    Mat redBinary; redBinary = LaneAlgo.getRedBinary().clone();

    Mat mergeImage; mergeImage = LaneAlgo.getMergeImage().clone();

    Mat histImage; histImage = LaneAlgo.getHistImage().clone();

    Mat maskImage; maskImage = LaneAlgo.getMaskImage().clone();

    Mat warpMask;  warpMask = LaneAlgo.getWarpMask().clone();

    Mat finalResult; finalResult = LaneAlgo.getFinalResult().clone();

    //imshow("resultado",finalResult);
    fun.ReadImage("resultado",finalResult);



    //===========Finish Real Time Processing===========

    waitKey(0);
    return 0;

}
