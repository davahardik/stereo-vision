#include <string>
#include<iostream>
#include<opencv2/core/cvstd.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include<opencv2/highgui.hpp>
#include "quasidensestereo.h"

using namespace std;

char* FindFileName(char* filename)
{
  char* result = strrchr(filename, '\\');
  return result ? result + 1 : filename;
}

int main(int argc, char** argv)
{

    char *img1name, *img2name;
    img1name = "im0.png";
    img2name = "im1.png";
    char filename[70];
    strcpy(filename, FindFileName(img1name));
    char winname[70];

    IplImage *img1, *img2, *disp;

    int64 t0 = cv::getTickCount();

    img1 = cvLoadImage(img1name, CV_LOAD_IMAGE_GRAYSCALE);
    img2 = cvLoadImage(img2name, CV_LOAD_IMAGE_GRAYSCALE);
    //cvNamedWindow("0", CV_WINDOW_AUTOSIZE );
    //cvShowImage("0", img1);

    QuasiDenseStereo qds;

    qds.initialise(cvGetSize(img1));

    qds.Param.BorderX = 15;				// borders around the image
    qds.Param.BorderY = 15;
    qds.Param.N = 5;						// neighbours
    qds.Param.Ct = 5.0000000000000000e-01;					// corre threshold for seeds
    qds.Param.Dg = 1;					// disparity gradient
    qds.Param.WinSizeX = 5;				// corr window size
    qds.Param.WinSizeY = 5;
    qds.Param.Tt = 200;				// propagation disparity
    int displvl = 64;				// disparity level
    sprintf(winname, "%s N=%i Ct=%g Dg=%i Wxy=%i Tt=%g Dlvl=%i",
        filename, qds.Param.N, qds.Param.Ct, qds.Param.Dg, qds.Param.WinSizeX,
        qds.Param.Tt, displvl);

    cout << "init ok" << endl;
    qds.process(img1,img2);

    cout << "process ok" << endl;

    disp = cvCreateImage(cvSize(qds.costWidth, qds.costHeight),IPL_DEPTH_8U,3);
    qds.getDisparityImage(disp, displvl);
    cvNamedWindow(winname, CV_WINDOW_AUTOSIZE );
    cvShowImage(winname, disp);

    int64 t1 = cv::getTickCount();
    double secs = (t1-t0)/cv::getTickFrequency();
    cout<<secs<<endl;
    cvWaitKey(0);

}
