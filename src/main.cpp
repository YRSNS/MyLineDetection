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



std::string PATH = "/home/yhon/Dev/MyLineDetection/img/calibrate/";

camera cam(PATH,2);
//funciones fun;
//thresholds th(PATH);

int sliderValue = 0;
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
    videoSize = Size((int)laneVideo.get(CAP_PROP_FRAME_WIDTH),
                     (int)laneVideo.get(CAP_PROP_FRAME_HEIGHT));


    //--------------Camera Calibration Start-----------------
    FileStorage fsRead;
    fsRead.open("Intrinsic.xml", FileStorage::READ);
    Mat src = imread(PATH+"calibration2.jpg");
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

    //Display Video Image
    laneVideo.set(CAP_PROP_POS_FRAMES, 0);
    laneVideo >> videoFrame;
    namedWindow("Original Image", WINDOW_NORMAL);
    imshow("Original Image",videoFrame);


    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();



    //Start Homography
    warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);
    namedWindow("Undistorsion", WINDOW_NORMAL);
    imshow("Undistorsion",videoFrameUndistorted);

    //Applying lane detection algorithm
    laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
    LaneAlgo.laneDetctAlgo();


    Mat warpEdge;
    warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();

    Mat imageRedChannel;
    imageRedChannel = LaneAlgo.getRedChannel().clone();

    Mat redBinary;
    redBinary = LaneAlgo.getRedBinary().clone();

    Mat mergeImage;
    mergeImage = LaneAlgo.getMergeImage().clone();

    Mat histImage;
    histImage = LaneAlgo.getHistImage().clone();

    Mat maskImage;
    maskImage = LaneAlgo.getMaskImage().clone();

    Mat warpMask;
    warpMask = LaneAlgo.getWarpMask().clone();

    Mat finalResult;
    finalResult = LaneAlgo.getFinalResult().clone();



    //To create debug window
    Mat debugWindowROI;
    Mat resizePic;

    //===========Start Real Time Processing===========

    float laneDistant = 0;
    stringstream ss;
    namedWindow("Real Time Execution", WINDOW_NORMAL);
    laneVideo.set(CAP_PROP_POS_FRAMES, 0);
    laneVideo >> videoFrame;
    Mat showVideos(videoFrame.size().height*2, videoFrame.size().width * 2, CV_8UC3, Scalar(0,0,0));
    laneDetection LaneAlgoVideo(_videoFrameUndistorted, perspectiveMatrix);
    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();

    VideoWriter writer;
    writer.open("Results.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, videoSize);
    VideoWriter writer2;
    writer2.open("DEBUG.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, showVideos.size());


    while(!videoFrame.empty())
    {
        //Start Homography
        warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

        //Applying lane detection algorithm
        //if(videoFrameCount == 0)
        LaneAlgoVideo.laneDetctAlgo();
        finalResult = LaneAlgoVideo.getFinalResult();

        //Detect the distance to lane center.
        laneDistant = LaneAlgoVideo.getLaneCenterDist();
        if(laneDistant > 0)
        {
            ss.str("");
            ss.clear();
            ss << abs(laneDistant) << "m " << " To the Right";
            putText(finalResult, ss.str(), Point(50,50), 0, 2, Scalar(0, 0, 255), 2);
        }
        else
        {
            ss.str("");
            ss.clear();
            ss << abs(laneDistant) << "m " << " To the Left";
            putText(finalResult, ss.str(), Point(50,50), 0, 2, Scalar(0, 0, 255), 2);
        }


        debugWindowROI = showVideos(Rect(0,0,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, videoFrame, 1, 0, debugWindowROI);

        debugWindowROI = showVideos(Rect(videoFrame.size().width,0,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, finalResult, 1, 0, debugWindowROI);

        mergeImage = LaneAlgoVideo.getMergeImage().clone();
        debugWindowROI = showVideos(Rect(0,videoFrame.size().height,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, mergeImage, 1, 0, debugWindowROI);

        debugWindowROI = showVideos(Rect(videoFrame.size().width,videoFrame.size().height,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, videoFramePerspective, 1, 0, debugWindowROI);


        imshow("Real Time Execution", showVideos);

        //write the video
        writer.write(finalResult);
        writer2.write(showVideos);

        laneVideo >> videoFrame;
        if(videoFrame.empty()) break;

        //videoFrameCount = (videoFrameCount + 1) % 10;

        //Calibration
        undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
        _videoFrameUndistorted = videoFrameUndistorted.clone();
        LaneAlgoVideo.setInputImage(_videoFrameUndistorted);

        if(waitKey(10) == 27) break;
    }


    //===========Finish Real Time Processing===========

    waitKey(0);
    return 0;

}
