//camera calibration
#include<iostream>
#include<opencv2/calib3d.hpp>
#include<opencv2/core/types.hpp>
#include<opencv2/core/persistence.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core/types_c.h>

class camera {
   public:
    camera(std::string dir,int dev):path(dir),device(dev) {}

    void mostrar(){
        std::cout<<"como estas"<<std::endl;
    }

    static void calcChessboardCornes(cv::Size boardSize,float squareSize, std::vector<cv::Point3f>& corners)
    {
        corners.clear();
        for(int i=0; i<boardSize.height; i++)
            for(int j=0; j<boardSize.width; j++)
                corners.push_back(cv::Point3f(float(j*squareSize),float(i*squareSize),0));

    }

    static void saveparams(const std::string& filename,const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                           const std::vector<cv::Mat>& rvecs,const std::vector<cv::Mat>& tvecs,const  double& RMS)
    {
        cv::FileStorage fs(filename,cv::FileStorage::WRITE);
        fs<<"Calibrate_Accuracy" << RMS;
        fs<<"Camera_Matrix" << cameraMatrix;
        fs<<"Distortion_Coefficients" << distCoeffs;
        fs<<"Rotation_vector" << rvecs;
        fs<<"Traslation_vector" << tvecs;

        if(!rvecs.empty() && !tvecs.empty())
        {
            CV_Assert(rvecs[0].type() == tvecs[0].type());
            cv::Mat bigmat((int)rvecs.size(),6,rvecs[0].type());
            for(int i=0; i<(int)rvecs.size(); i++)
            {
                cv::Mat r= bigmat(cv::Range(i,i+1),cv::Range(0,3));
                cv::Mat t= bigmat(cv::Range(i,i+1),cv::Range(0,3));

                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);

                r = rvecs[i].t();
                t = tvecs[i].t();
            }

            fs << "Rotation vector + Translation vector ";
            fs << "extrinsic_parameters"<<bigmat;
        }

        fs.release();
    }

    void InitRun()
    {
        calcChessboardCornes(patternsize,18,corners3D);
        cv::Mat img, imgGray;
        bool found;

        for(int i=0; i<14; i++)
        {
            imgs << path+"Cam" << i <<".jpg";
            img = cv::imread(imgs.str().c_str());
            cv::cvtColor(img,imgGray,cv::COLOR_BGR2GRAY);
            imgs = std::stringstream();

            found = cv::findChessboardCorners(imgGray,patternsize,corners2D,cv::CALIB_CB_ADAPTIVE_THRESH+
                                                                            cv::CALIB_CB_NORMALIZE_IMAGE+
                                                                            cv::CALIB_CB_FAST_CHECK);
            if(found)
            {
                cv::cornerSubPix(imgGray,corners2D,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(
                                                                                cv::TermCriteria::EPS+
                                                                                cv::TermCriteria::COUNT,30,0.1));
                cv::drawChessboardCorners(img,patternsize,cv::Mat(corners2D),found);
                coord2D.push_back(corners2D);
                coord3D.push_back(corners3D);
            }
            cv::namedWindow("image",cv::WINDOW_AUTOSIZE);
            cv::imshow("image",img);
            cv::waitKey(1500);
        }
        cv::destroyWindow("image");

        cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
        cv::Mat distCoeffs = cv::Mat::zeros(8,1,CV_64F);
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;

        double rms = cv::calibrateCamera(coord3D,coord2D,img.size(),cameraMatrix,
                                         distCoeffs,rvecs,tvecs,
                                         cv::CALIB_FIX_PRINCIPAL_POINT+
                                         cv::CALIB_FIX_ASPECT_RATIO+
                                         cv::CALIB_ZERO_TANGENT_DIST,cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,2.22e-16));

        std::cout<< "RMS: " << rms << std::endl;
        std::cout<< "Camera matrix: " << cameraMatrix << std::endl;
        std::cout<< "Distortion: " << distCoeffs << std::endl;

        //saveparams("/home/yrsn/Dev/cpp/CV_2022/vnode/cam/DataCam.yml",cameraMatrix,distCoeffs,rvecs,tvecs,rms);

        cv::Mat imageUndistorted,image;
        cv::VideoCapture cap;
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 400); // valueX = your wanted width
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 300); // valueY = your wanted heigth
        cap.open(device);
        cap.set(cv::CAP_PROP_FPS, 30);


        cv::namedWindow("imgOriginal",cv::WINDOW_AUTOSIZE);
        cv::namedWindow("imgCalibrada",cv::WINDOW_AUTOSIZE);

        cv::Mat rsize;
        cv::Mat tsize;

        while(1)
        {
            cap >> image;
            cv::undistort(image,imageUndistorted,cameraMatrix,distCoeffs);
            //resize image to nice show
            //cv::resize(imageUndistorted,rsize,cv::Size(400,300),cv::INTER_LINEAR);
            //cv::resize(image,tsize,cv::Size(400,300),cv::INTER_LINEAR);

            cv::imshow("imgOriginal",image);
            cv::imshow("imgCalibrada",imageUndistorted);
            cv::waitKey(200);
        }
        cv::waitKey(0);
        getchar();
        return;
    }

   private:
      std::string path;
      // parameters to test program
      cv::Size patternsize = cv::Size(9,6);// esquinas inferiores del teblero
      std::vector<cv::Point3f> corners3D;  //
      std::vector<cv::Point2f> corners2D;  // guarda puntos del tableto aqui
      std::vector<std::vector<cv::Point2f>> coord2D; // Ubicacion de las esquinas detectadas en la imagen
      std::vector<std::vector<cv::Point3f>> coord3D; // Ubicacion real de los puntos3D

      //imagen
      std::stringstream imgs;
      int device;
};
