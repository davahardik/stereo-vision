#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ximgproc.hpp"

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include<sgbm.cpp>
#include<sgbmbinary.cpp>

#include "elas.h"
#include "image.h"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using namespace pcl;

bool video_file=true;
bool image_file=false;

void splitPoints(const std::string& inputStr, std::vector<std::string>& splitVec)
{
    std::size_t pos = 0, found;
     while((found = inputStr.find_first_of(' ', pos)) != std::string::npos) {

        splitVec.push_back(inputStr.substr(pos, found - pos));
        pos = found+1;
     }

     splitVec.push_back(inputStr.substr(pos));
}

void loadGT(std::string &filename,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::ifstream pointsFile;
    pointsFile.open(filename);

    std::string line;
    std::vector<std::string> tmpLineCont;
    std::vector<std::vector<float> > pts;

    if (pointsFile.is_open())
    {
      while ( std::getline (pointsFile,line) )
      {
       tmpLineCont.clear();
       splitPoints(line, tmpLineCont);
       if ( tmpLineCont.size() == 3) {
          float strToDouble0 = std::atof(tmpLineCont[0].c_str());
          float strToDouble1 = std::atof(tmpLineCont[1].c_str());
          float strToDouble2 = std::atof(tmpLineCont[2].c_str());

          std::vector<float> tmpDoubleVec;
          tmpDoubleVec.push_back(strToDouble0);
          tmpDoubleVec.push_back(strToDouble1);
          tmpDoubleVec.push_back(strToDouble2);

          pts.push_back(tmpDoubleVec);
       }
      }
      pointsFile.close();
    }
   pcl::PointXYZ tmpPoint;
    for (int i = 0; i < pts.size(); ++i) {

          tmpPoint.x = pts[i][0];
          tmpPoint.y = pts[i][1];
          tmpPoint.z = pts[i][2];
          cloud->points.push_back(tmpPoint);
     }

}


void publishPointCloud(Mat& left, Mat& dmap,Mat Q, Mat R, Mat T, ofstream& obj_file) {
  Mat V = Mat(4, 1, CV_64FC1);
  Mat pos = Mat(4, 1, CV_64FC1);
  int ndisp = 64;
  for (int i = 0; i < dmap.cols; i++) {
    for (int j = 0; j < dmap.rows; j++) {
      int d = dmap.at<uchar>(j,i) * ((float)ndisp / 255.);
      // if low disparity, then ignore
      if (d < 2)
        continue;
      // V is the vector to be multiplied to Q to get
      // the 3D homogenous coordinates of the image point
      V.at<double>(0,0) = (double)(i);
      V.at<double>(1,0) = (double)(j);
      V.at<double>(2,0) = (double)d;
      V.at<double>(3,0) = 1.;
      pos = Q * V; // 3D homogeneous coordinate
      double X = pos.at<double>(0,0) / pos.at<double>(3,0);
      double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
      double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
      Mat point3d_cam = Mat(3, 1, CV_64FC1);
      point3d_cam.at<double>(0,0) = X;
      point3d_cam.at<double>(1,0) = Y;
      point3d_cam.at<double>(2,0) = Z;
      // transform 3D point from camera frame to robot frame
      Mat point3d_robot = R * point3d_cam + T;
      int r = (int)left.at<Vec3b>(j,i)[2];
      int g = (int)left.at<Vec3b>(j,i)[1];
      int b = (int)left.at<Vec3b>(j,i)[0];
      obj_file << "v " << point3d_robot.at<double>(0,0) << " " <<
      point3d_robot.at<double>(1,0) << " " << point3d_robot.at<double>(2,0);
      obj_file << " " << r << " " << g << " " << b << endl;
    }
  }
}
// compute disparities of pgm image input pair file_1, file_2
void process (const char* file_1,const char* file_2) {

  cout << "Processing: " << file_1 << ", " << file_2 << endl;

  // load images
  image<uchar> *I1,*I2;
  I1 = loadPGM(file_1);
  I2 = loadPGM(file_2);

  // check for correct size
  if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
      I1->width()!=I2->width() || I1->height()!=I2->height()) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1->width() <<  " x " << I1->height() <<
                 ", I2: " << I2->width() <<  " x " << I2->height() << endl;
    delete I1;
    delete I2;
    return;
  }

  // get image width and height
  int32_t width  = I1->width();
  int32_t height = I1->height();

  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,width}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1->data,I2->data,D1_data,D2_data,dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  float disp_max = 0;
  for (int32_t i=0; i<width*height; i++) {
    if (D1_data[i]>disp_max) disp_max = D1_data[i];
    if (D2_data[i]>disp_max) disp_max = D2_data[i];
  }

  // copy float to uchar
  image<uchar> *D1 = new image<uchar>(width,height);
  image<uchar> *D2 = new image<uchar>(width,height);
  for (int32_t i=0; i<width*height; i++) {
    D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
    D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
  }

  // save disparity images
  char output_1[1024];
  char output_2[1024];
  strncpy(output_1,file_1,strlen(file_1)-4);
  strncpy(output_2,file_2,strlen(file_2)-4);
  output_1[strlen(file_1)-4] = '\0';
  output_2[strlen(file_2)-4] = '\0';
  strcat(output_1,"_disp.pgm");
  strcat(output_2,"_disp.pgm");
  savePGM(D1,output_1);
  savePGM(D2,output_2);

  // free memory
  delete I1;
  delete I2;
  delete D1;
  delete D2;
  free(D1_data);
  free(D2_data);
}

int main(int argc, char *argv[])
{
    Mat groundtruth;
    std::string filename="/home/hd/Desktop/Thesis Codes/build-HamlynStereo-Desktop_Qt_5_8_0_GCC_64bit-Debug/heartDepthMap_0.txt";

    pcl::PointCloud<pcl::PointXYZ>::Ptr GT (new pcl::PointCloud<pcl::PointXYZ>);

    //load selected points from txt file
    loadGT(filename,GT);

    pcl::io::savePLYFile("groundtruth.ply",*GT);

    cv::Mat P1 ;
    cv::Mat P2;
    cv::Mat Q;
    cv::Mat R1 ;
    cv::Mat R2 ;

    Mat M1 = (Mat_<double>(3,3) << 391.656525, 0.000000 ,165.964371, 0.000000, 426.835144, 154.498138, 0.000000, 0.000000, 1.000000);
    Mat D1 = (Mat_<double>(4,1) << -0.196312, 0.129540, 0.004356, 0.006236);

    Mat M2 = (Mat_<double>(3,3) << 390.376862, 0.000000 ,190.896454, 0.000000, 426.228882, 145.071411,0.000000, 0.000000, 1.000000);
    Mat D2 = (Mat_<double>(4,1) << -0.205824, 0.186125, 0.015374 ,0.003660);

    Mat R = (Mat_<double>(3,3) << 0.999999, -0.001045, -0.000000,0.001045, 0.999999, -0.000000,0.000000 ,0.000000, 1.000000);
    Mat T = (Mat_<double>(3,1) << -5.520739, -0.031516, -0.051285);

    Mat left,right;
    string left_img, right_img;


    if(video_file==true)
    {
    VideoCapture Left("/home/hd/Desktop/Thesis Codes/build-HamlynStereo-Desktop_Qt_5_8_0_GCC_64bit-Debug/L.avi");
    VideoCapture Right("/home/hd/Desktop/Thesis Codes/build-HamlynStereo-Desktop_Qt_5_8_0_GCC_64bit-Debug/R.avi");


    for(int i=0;i<1;i++){

       Mat frameL;
       Left >> frameL;

       Mat frameR;
       Right >> frameR;

       left=frameL.clone();
       right=frameR.clone();

     }

    Left.release();
    Right.release();


    imshow("Left",left);
    imshow("Right",right);

    }

    if(image_file==true)
    {
        left_img="left.pgm";
        right_img="right.pgm";

        left=imread(left_img);
        right=imread(right_img);

        imshow("Left",left);
        imshow("Right",right);
    }

    Mat imgU1,imgU2;

    Size Imagesize=left.size();

    cv::Mat lmapx, lmapy, rmapx, rmapy;
    stereoRectify(M1, D1, M2, D2, Imagesize, R, T, R1, R2, P1, P2, Q,CALIB_ZERO_DISPARITY );

    cv::initUndistortRectifyMap(M1, D1, R1, P1,Imagesize, CV_32FC1, lmapx, lmapy);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, Imagesize, CV_32FC1, rmapx, rmapy);
    cv::remap(left, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
    cv::remap(right, imgU2, rmapx, rmapy, cv::INTER_LINEAR);

    imshow("Left",imgU1);
    imshow("Right",imgU2);

    Mat left_gray, right_gray;

    cvtColor(imgU1,left_gray,CV_BGR2GRAY);
    cvtColor(imgU2,right_gray,CV_BGR2GRAY);

    imwrite("left.pgm",left_gray);
    imwrite("right.pgm",right_gray);


    imwrite("left.png",imgU1);
    imwrite("right.png",imgU2);




    stereosgbm(left_gray,right_gray,imgU1,imgU2,Q);

    Mat left_image=imread("left.png",CV_8UC1);
    Mat right_image=imread("right.png",CV_8UC1);

    stereobinary(left_image,right_image,left_image,right_image,Q);

    process("left.pgm", "right.pgm");

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudelas (new pcl::PointCloud<pcl::PointXYZ>);

    Mat disp_elas=imread("left_disp.pgm",IMREAD_UNCHANGED);
    disp_elas.convertTo(disp_elas, CV_8U);

    imshow("elas",disp_elas);

    ofstream obj_file;
    obj_file.open("elas.obj");
    publishPointCloud(imgU1, disp_elas, Q,R,T,obj_file);

    cout<<"done"<<endl;
    waitKey(0);
    return 0;

}
