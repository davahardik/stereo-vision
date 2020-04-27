
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <fstream>
#include <pcl/filters/filter.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/ximgproc.hpp"

#include<math.h>
#include <string>

 #include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>


#include <opencv2/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <pcl/common/transforms.h>
  #include <opencv2/core/eigen.hpp>
using namespace cv;
using namespace cv::ximgproc;
using namespace std;
using namespace pcl;


Mat disp_bm;
Mat disp_sgbm;
Mat disp_cbm;
Size imagesize;

Mat filtered_disp,solved_disp,solved_filtered_disp;
Mat filtered_dispbm;
double lambda = 8;
double sigma  =1.5;

int max_disparity=64;



Mat CIE(Mat image) {


    Mat grayScaleImage(image.size(),CV_8UC1);
    int rows=image.rows;
    int cols=image.cols;


    for(int i=0;i<rows;i++){
     for(int j=0;j<cols;j++){

      Vec3b intensity=image.at<Vec3b>(i,j);

      int blue=intensity.val[0];
      int green =intensity.val[1];
      int red=intensity.val[2];

      grayScaleImage.at<uchar>(i,j)=blue*0.0722+green*0.7152+red*0.2126;

     }

    }
 return grayScaleImage;
}

Mat naivemean(Mat image) {


    Mat grayScaleImage(image.size(),CV_8UC1);
    int rows=image.rows;
    int cols=image.cols;


     for(int i=0;i<rows;i++){
     for(int j=0;j<cols;j++){

      Vec3b intensity=image.at<Vec3b>(i,j);

      int blue=intensity.val[0];
      int green =intensity.val[1];
      int red=intensity.val[2];

      grayScaleImage.at<uchar>(i,j)=(blue+green+red)/3;

     }

    }

     return grayScaleImage;
}

Mat stereo(Mat left_gray,Mat right_gray,Mat left,Mat right){

    Mat left_disp,right_disp;
    int winsize=5;

  //Stereo Semi-global matching + WLS filter
    Ptr<StereoSGBM> sgbm  = StereoSGBM::create(0,144,winsize);

    Ptr<DisparityWLSFilter> wls_filter;

    wls_filter = createDisparityWLSFilter(sgbm);
    Ptr<StereoMatcher> sgbm_R = createRightMatcher(sgbm);

    int mindisp=0;
    int P1=200;
    int P2=1000;
    int prefiltcap=20;
    int specrange=15;
    int specwinsize=5;
    int uniqueness=12;
    int disp12maxdiff=2;
    int lambda=60;
    int sigma=2;
    int LRthresh=2;

    FileStorage fs2("sgbmparameter.yml", FileStorage::WRITE);

    fs2<<"MinDisp" << mindisp;
    fs2<<"WinSize" << winsize;
    fs2<<"P1" << P1;
    fs2<<"P2" << P2;
    fs2<<"PreFiltCap" << prefiltcap;
    fs2<<"Uniqueness" << uniqueness;
    fs2<<"Disp12MaxDiff" << disp12maxdiff;
    fs2<<"SpecRange" << specrange;
    fs2<<"SpecWinSize" << specwinsize;
    fs2<<"Lambda" << lambda;
    fs2<<"Sigma" << sigma;

    fs2.release();


    sgbm->setMinDisparity(mindisp);

    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(prefiltcap);
    sgbm->setMode(2);
    sgbm->setDisp12MaxDiff(disp12maxdiff);
    sgbm->setUniquenessRatio(uniqueness);
    sgbm->setSpeckleRange(specrange);
    sgbm->setSpeckleWindowSize(specwinsize);


    sgbm-> compute(  left_gray, right_gray,left_disp);
    sgbm_R-> compute(right_gray, left_gray,right_disp);

    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    wls_filter->filter(left_disp,left,filtered_disp,right_disp);
    wls_filter->setLRCthresh(LRthresh);

    // MEDIAN AND GAUSSIAN FILTERING PARAMETERS

    // Filter kernel size
    int nKernelSize = 9;



   // filtered_disp.convertTo(filtered_disp, CV_8U);
  //  medianBlur(filtered_disp, filtered_disp, nKernelSize);
   filtered_disp.convertTo(filtered_disp, CV_32F);

  // GaussianBlur(filtered_disp, filtered_disp, Size(nKernelSize, nKernelSize),0);

    Mat disp;
    cv::normalize(filtered_disp,disp,0,255,cv::NORM_MINMAX,CV_8U);



    return disp;

    disp.release();
    filtered_disp.release();
    left_gray.release();
    right_gray.release();
    left.release();
    left_disp.release();
    right_disp.release();
}

//Color Point Cloud takes more time.....
void gencloud(Mat disp,Mat Q,Mat left,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud){


    pointcloud->width = static_cast<uint32_t>(disp.cols);
    pointcloud->height = static_cast<uint32_t>(disp.rows);
    pointcloud->is_dense = false;


    Mat depthmap;

    // Perform stereo reconstruction of point cloud in camera coordinate system (origin in left camera center, z axis oriented according to line of sight)
    reprojectImageTo3D(disp, depthmap, Q, true, CV_32F);

   //Mat to PointCloud conversion ::::

       pcl::PointXYZRGB point;

       for (int i = 0; i < disp.rows; i+=3)
           {
               uchar* rgb_ptr = left.ptr<uchar>(i);
               uchar* disp_ptr = disp.ptr<uchar>(i);
               double* xyz_ptr = depthmap.ptr<double>(i);

               for (int j = 0; j < disp.cols; j+=3)
               {
                   if (!disp.at<uchar>(i, j)==0){
                   uchar d = disp_ptr[j];
                   if (d == 0) continue;
                   Point3f p = depthmap.at<Point3f>(i, j);

                   point.z = p.z;   // I have also tried p.z/16
                   point.x = p.x;
                   point.y = p.y;



                  point.b = rgb_ptr[3*j];
                  point.g = rgb_ptr[3*j + 1];
                  point.r = rgb_ptr[3*j + 2];

                   // pointcloud->points.push_back(point);


                   if (point.z >= 0 && point.z <=150)
                     {
                           pointcloud->points.push_back(point);
                      }

                   }

               }
           }

    pointcloud->width = 1;
    pointcloud->height = pointcloud->points.size();
/*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (pointcloud);
    sor.setMeanK (1000);
    sor.setStddevMulThresh (4);
    sor.filter (*pointcloud);
*/
    disp.release();
    depthmap.release();
    Q.release();

}






int main(int argc, char *argv[])
{

    cv::Mat M1 ;
    cv::Mat D1 ;
    cv::Mat M2;
    cv::Mat D2;

    cv::Mat P1 ;
    cv::Mat P2;
    cv::Mat Q,R,T;
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

    Mat imgU1, imgU2;

    //load image file
    cv::Mat img = cv::imread("0.png");

    Mat dst,img_gray;

    cvtColor( img, img_gray, COLOR_BGR2GRAY ); // Convert the image to Gray
    threshold( img_gray, dst, 32, 255, CV_THRESH_BINARY_INV );

   img.setTo(Scalar(0,0,0), dst);

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

    Size Imagesize=Left.size();

    cv::Mat lmapx, lmapy, rmapx, rmapy;
    stereoRectify(M1, D1, M2, D2, Imagesize, R, T, R1, R2, P1, P2, Q,CALIB_ZERO_DISPARITY );

    cv::initUndistortRectifyMap(M1, D1, R1, P1,Imagesize, CV_32FC1, lmapx, lmapy);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, Imagesize, CV_32FC1, rmapx, rmapy);
    cv::remap(Left, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
    cv::remap(Right, imgU2, rmapx, rmapy, cv::INTER_LINEAR);

    Mat left_gray, right_gray;


    double start = (double)cv::getTickCount();

    cvtColor(imgU1,left_gray,CV_BGR2GRAY);
    cvtColor(imgU2,right_gray,CV_BGR2GRAY);

    Mat opencvdisp=stereo(left_gray,right_gray,imgU1,imgU2);
   // imshow("Opencvgray disp",opencvdisp);


    /*

    cv::Rect roicloud;
    roicloud.x = 310;
    roicloud.y =300;
    roicloud.width=190;
    roicloud.height = 144 ;
    opencvdisp=opencvdisp(roicloud);

*/

  //  pcl::visualization::PCLVisualizer viewer(" Point Cloud Datsets Visualizer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudopencv (new pcl::PointCloud<pcl::PointXYZRGB>);

    gencloud(opencvdisp,Q,imgU1,pointcloudopencv);



    pcl::io::savePLYFile("m.ply",*pointcloudopencv);




    //wait for 40 milliseconds
cvWaitKey(0);
return(0);

}

