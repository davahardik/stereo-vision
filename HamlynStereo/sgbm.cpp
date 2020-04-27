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

using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using namespace pcl;

int mindisp=0;
int ndisp=64;
int nwin=5;

void stereosgbm(Mat left_gray,Mat right_gray,Mat left,Mat right,Mat Q){



    Mat filtered_disp;
    Mat left_disp,right_disp;

    Ptr<StereoSGBM> sgbm  = StereoSGBM::create(0,ndisp,nwin);

    Ptr<DisparityWLSFilter> wls_filter;

    wls_filter = createDisparityWLSFilter(sgbm);
    Ptr<StereoMatcher> sgbm_R = createRightMatcher(sgbm);

    sgbm->setMinDisparity(0);
    sgbm->setP1(20);
    sgbm->setP2(100);
    sgbm->setPreFilterCap(20);
    sgbm->setMode(2);
    sgbm->setDisp12MaxDiff(2);
    sgbm->setUniquenessRatio(12);
    sgbm->setSpeckleRange(15);
    sgbm->setSpeckleWindowSize(5);

    sgbm-> compute(  left_gray, right_gray,left_disp);
    sgbm_R-> compute(right_gray, left_gray,right_disp);

    wls_filter->setLambda(60);
    wls_filter->setSigmaColor(2);
    wls_filter->filter(left_disp,left,filtered_disp,right_disp);
    wls_filter->setLRCthresh(3);

    Mat disp;
    filtered_disp.convertTo(disp, CV_32F);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pointcloud->width = static_cast<uint32_t>(disp.cols);
    pointcloud->height = static_cast<uint32_t>(disp.rows);
    pointcloud->is_dense = false;

    Mat depthmap;

    // Perform stereo reconstruction of point cloud in camera coordinate system (origin in left camera center, z axis oriented according to line of sight)
    reprojectImageTo3D(disp, depthmap, Q, true, CV_32F);

   //Mat to PointCloud conversion ::::

       pcl::PointXYZRGB point;

       for (int i = 0; i < disp.rows; i++)
           {
               uchar* rgb_ptr = left.ptr<uchar>(i);
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

                   point.b = rgb_ptr[3*j];
                   point.g = rgb_ptr[3*j + 1];
                   point.r = rgb_ptr[3*j + 2];

                 // pointcloud->points.push_back(point);

                  if (point.z >= 0 && point.z <=100)
                    {
                          pointcloud->points.push_back(point);
                    }

                   }

               }
           }

    pointcloud->width = 1;
    pointcloud->height = pointcloud->points.size();

    pcl::io::savePLYFile("SGBM.ply",*pointcloud);


}
