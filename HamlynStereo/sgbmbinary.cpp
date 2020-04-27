#include "opencv2/stereo.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/thread/thread.hpp>
#include<opencv2/ximgproc.hpp>

#include<fastbilateral.h>

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace cv::stereo;
using namespace cv::ximgproc;

void stereobinary(Mat left_gray,Mat right_gray,Mat left,Mat right,Mat Q){

    int mindisp=0;
    int ndisp=64;
    int nwin=9;

    Mat filtered_disp;

    Mat left_disp,right_disp;

    Mat imgDisparity16S2 = Mat(left_gray.rows, left_gray.cols, CV_16S);
    Mat imgDisparity8U2 = Mat(left_gray.rows, left_gray.cols, CV_8UC2);

    Ptr<StereoBinarySGBM> sgbm = StereoBinarySGBM::create(0, ndisp,nwin);

    sgbm->setP1(20);
    sgbm->setP2(50);
    sgbm->setMinDisparity(0);
    sgbm->setUniquenessRatio(5);
    sgbm->setSpeckleRange(15);
    sgbm->setSpeckleWindowSize(5);

    sgbm->setDisp12MaxDiff(2);
    sgbm->setBinaryKernelType(CV_MEAN_VARIATION);
    sgbm->setSpekleRemovalTechnique(CV_SPECKLE_REMOVAL_ALGORITHM);
    sgbm->setSubPixelInterpolationMethod(CV_SIMETRICV_INTERPOLATION);
    sgbm->compute(left_gray, right_gray, imgDisparity16S2);


    sgbm->setMode(0);

  imgDisparity16S2.convertTo(imgDisparity8U2, CV_8UC1);
         //   imshow("Windowsgm", imgDisparity8U2);

   cv_extend::bilateralFilter(imgDisparity8U2,imgDisparity8U2, 3,3);

   imshow("Windowsgm", imgDisparity8U2);


    Mat disp;
    imgDisparity8U2.convertTo(disp, CV_32F);


    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZ>);

    pointcloud->width = static_cast<uint32_t>(disp.cols);
    pointcloud->height = static_cast<uint32_t>(disp.rows);
    pointcloud->is_dense = true;

    Mat depthmap;

    // Perform stereo reconstruction of point cloud in camera coordinate system (origin in left camera center, z axis oriented according to line of sight)
    reprojectImageTo3D(disp, depthmap, Q, true, CV_32F);

   //Mat to PointCloud conversion ::::

       pcl::PointXYZ point;

       for (int i = 0; i < disp.rows; i++)
           {

               uchar* disp_ptr = disp.ptr<uchar>(i);
               double* xyz_ptr = depthmap.ptr<double>(i);

               for (int j = 0; j < disp.cols; j++)
               {
                   if (!disp.at<uchar>(i, j)==0){
                   uchar d = disp_ptr[j];
                   if (d == 0) continue;
                   Point3f p = depthmap.at<Point3f>(i, j);

                   point.z = p.z*16;
                   point.x = p.x*16;
                   point.y = p.y*16;

                 pointcloud->points.push_back(point);
/*
                  if (point.z >= 0 && point.z <=150)
                    {
                          pointcloud->points.push_back(point);
                    }
*/
                   }

               }
           }

    pointcloud->width = 1;
    pointcloud->height = pointcloud->points.size();

    pcl::io::savePLYFile("SGBMBinary.ply",*pointcloud);


      }




