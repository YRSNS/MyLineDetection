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
#include "threshold.h"

using namespace std;
using namespace cv;

std::string PATH = "/home/y3rsn/Dev/cpp/CV_2022/vnode/img/";

camera cam(PATH,2);
funciones fun;
thresholds th(PATH);


int main()
{
    //cam.InitRun();
    //ReadVideo();
    //fun.ReadImage(PATH);
    th.get_combined_gradients();

    destroyAllWindows();

    return 0;
}

