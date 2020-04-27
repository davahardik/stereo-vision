#include "opencv2/stereo.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>

#include<pcl/filters/bilateral.h>
#include<pcl/filters/fast_bilateral.h>
#include<pcl/filters/fast_bilateral_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/median_filter.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/ply_io.h>



#include <pcl/conversions.h>
#include "opencv2/ximgproc.hpp"

#include <opencv2/tracking.hpp>
#include "fastbilateral.h"


using namespace std;
using namespace cv;
using namespace cv::stereo;
using namespace pcl;
using namespace cv::ximgproc;
int main(int argc, char** argv)
{

    cv::Mat M1 ;
    cv::Mat D1 ;
    cv::Mat M2;
    cv::Mat D2;

    cv::Mat P1 ;
    cv::Mat P2;
    Mat Q,R,T;
    cv::Mat R1 ;
    cv::Mat R2 ;



    FileStorage fs("/home/hd/Desktop/Thesis Codes/build-Stereocameracalibration-Desktop_Qt_5_8_0_GCC_64bit-Debug/calibration.yml", FileStorage::READ);
    fs["CameraMatrixLeft"] >> M1;
    fs["CameraMatrixRight"] >> M2;
    fs["DistortionCoefficientsLeft"] >> D1;
    fs["DistortionCoefficientsRight"] >> D2;
    fs["RotationMatrix"] >> R;
    fs["TranslationVector"] >> T;

    int w=712;
    int h=712;

    Size Imagesize=Size(w,h);
    Mat imgU1, imgU2;

    //load image file
    cv::Mat img = cv::imread("/home/hd/Desktop/Thesis Codes/build-SGBMBINARY-Desktop_Qt_5_8_0_GCC_64bit-Debug/0.png");



    //crop image for virtual left and right camera view
    cv::Rect roiL;
    roiL.x = 56;
    roiL.y = 43;
    roiL.width = 712;
    roiL.height =712 ;

    cv::Rect roiR;
    roiR.x = 919;
    roiR.y = 45;
    roiR.width =712;
    roiR.height = 712 ;

    cv::Mat Left= img(roiL);
    cv::Mat Right = img(roiR);

    cv::Mat lmapx, lmapy, rmapx, rmapy;
    stereoRectify(M1, D1, M2, D2, Imagesize, R, T, R1, R2, P1, P2, Q,CALIB_ZERO_DISPARITY );

    cv::initUndistortRectifyMap(M1, D1, R1, P1,Imagesize, CV_32FC1, lmapx, lmapy);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, Imagesize, CV_32FC1, rmapx, rmapy);
    cv::remap(Left, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
    cv::remap(Right, imgU2, rmapx, rmapy, cv::INTER_LINEAR);



    Mat left_gray, right_gray;
    Mat left_disp,right_disp;
    Mat norm_disp,filtered_disp;
    Mat depthmap;


    cvtColor(imgU1,  left_gray,  COLOR_BGR2GRAY);
    cvtColor(imgU2, right_gray, COLOR_BGR2GRAY);

    int wsize=5;


        // we set the corresponding parameters
        Ptr<StereoBinarySGBM> sgbm = StereoBinarySGBM::create(0, 128, 5);


        sgbm->setP1(200);
        sgbm->setP2(1000);
        sgbm->setMinDisparity(0);
        sgbm->setUniquenessRatio(5);
        sgbm->setSpeckleWindowSize(200);
        sgbm->setSpeckleRange(12);
        sgbm->setDisp12MaxDiff(2);
        sgbm->setBinaryKernelType(CV_DENSE_CENSUS);
        sgbm->setSpekleRemovalTechnique(CV_SPECKLE_REMOVAL_AVG_ALGORITHM);
        sgbm->setSubPixelInterpolationMethod(CV_SIMETRICV_INTERPOLATION);
        sgbm->compute(left_gray, right_gray, left_disp);


        Mat disp;
        cv::normalize(left_disp,disp,0,255,cv::NORM_MINMAX,CV_8U);


        int lambda=60;
        int sigma=2;
        fastGlobalSmootherFilter(left_gray,disp,filtered_disp,lambda,sigma);

   //Fast Bilatral Filter     https://cs.brown.edu/courses/cs129/2012/lectures/bf_course_Brown_Oct2012.pdf
     //   cv_extend::bilateralFilter(disp,disp, 10,10);
        cv::imshow("disparity", filtered_disp);


    // Perform stereo reconstruction of point cloud in camera coordinate system (origin in left camera center, z axis oriented according to line of sight)
       reprojectImageTo3D(disp, depthmap, Q, true, CV_32F);

   //Mat to PointCloud conversion ::::
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new   pcl::PointCloud<pcl::PointXYZRGB>());


       pointcloud->width = static_cast<uint32_t>(left_disp.cols);
       pointcloud->height = static_cast<uint32_t>(left_disp.rows);
       pointcloud->is_dense = false;
       pcl::PointXYZRGB point;

       for (int i = 0; i < left_disp.rows; i++)
           {
               uchar* rgb_ptr = imgU1.ptr<uchar>(i);
               uchar* disp_ptr = left_disp.ptr<uchar>(i);
               double* xyz_ptr = depthmap.ptr<double>(i);

               for (int j = 0; j < left_disp.cols; j++)
               {
                   uchar d = disp_ptr[j];
                   if (d == 0) continue;
                   Point3f p = depthmap.at<Point3f>(i, j);

                   point.z = p.z;   // I have also tried p.z/16
                   point.x = p.x;
                   point.y = p.y;

                   point.b = rgb_ptr[3 * j];
                   point.g = rgb_ptr[3 * j + 1];
                   point.r = rgb_ptr[3 * j + 2];

                  //pointcloud->points.push_back(point);

                  if (point.z >= 0 && point.z <=50)
                             {
                                 pointcloud->points.push_back(point);

                             }

               }
           }
       pointcloud->width = 1;
       pointcloud->height = pointcloud->points.size();

/*
       pcl::FastBilateralFilter<pcl::PointXYZRGB> filter;
       filter.setInputCloud(pointcloud);
       filter.setSigmaS(5);
       filter.setSigmaR(5e-3);
       filter.applyFilter(*pointcloud);
    pcl::io::savePLYFile ("/home/hd/Desktop/Thesis Codes/build-SGBMBINARY-Desktop_Qt_5_8_0_GCC_64bit-Debug/SGBMBinary.ply", *pointcloud);


     pcl::MedianFilter<pcl::PointXYZRGB>medi;
     medi.setInputCloud(pointcloud);
     medi.setWindowSize(5);
     medi.setMaxAllowedMovement(0.80);
     medi.applyFilter(*pointcloud);
*/
     pcl::visualization::PCLVisualizer viewer(" Point Cloud Datsets Visualizer");
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);

     viewer.addPointCloud(pointcloud, rgb,"source_cloud");
     viewer.initCameraParameters ();
     viewer.spin();




    waitKey(0);
    return 0;
}
