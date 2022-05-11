#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;


int main(int argc, char **argv)
{
    int l=5, w=7;
    float size=2;

    std::vector<cv::Point3d> posXadrez;
    for(int i=0; i<w; i++)
    {
        for(int j=0; j<l; j++)
            posXadrez.push_back(cv::Point3d(i*size,j*size,0));
    }

    std::vector<std::vector<cv::Point3d>> real_points;
    std::vector<std::vector<cv::Point2d>> all_cornersL, all_cornersR;

    cv::Mat imgL, imgR;

    for(int i=0; i<6; i++)
    {
        imgL = cv::imread("/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_ik/calib_imgs/L"+std::to_string(i)+".png");
        imgR = cv::imread("/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_ik/calib_imgs/R"+std::to_string(i)+".png");

        std::vector<cv::Point2f> cornersL, cornersR;
        bool doneL = cv::findChessboardCorners(imgL, cv::Size(l,w), cornersL);
        bool doneR = cv::findChessboardCorners(imgR, cv::Size(l,w), cornersR);

        if(doneL && doneR)
        {
            // cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
            // cv::cornerSubPix(imgL, cornersL, cv::Size(5,5), cv::Size(-1,-1), criteria);
            // cv::cornerSubPix(imgR, cornersR, cv::Size(5,5), cv::Size(-1,-1), criteria);

            // cv::drawChessboardCorners(imgL, cv::Size(l,w), cornersL, doneL);
            std::vector<cv::Point2d> cornersL_d, cornersR_d;
            for(int j=0; j<cornersL.size(); j++)
            {
                cornersL_d.push_back(cv::Point2d((double)cornersL[j].x, (double)cornersL[j].y));
                cornersR_d.push_back(cv::Point2d((double)cornersR[j].x, (double)cornersR[j].y));
            }

            real_points.push_back(posXadrez);
            all_cornersL.push_back(cornersL_d);
            all_cornersR.push_back(cornersR_d);
        }

        // std::cout << i << std::endl;
        // cv::imshow("imgL",imgL);
	    // cv::waitKey(0);

    }

    cv::Mat cameraMatrixL, cameraMatrixR, distCoeffsL, distCoeffsR, R, T, E, F;
    int flag = 0;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flag |= cv::fisheye::CALIB_CHECK_COND;
    flag |= cv::fisheye::CALIB_FIX_SKEW;

    cv::fisheye::stereoCalibrate(real_points, all_cornersL, all_cornersR, cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imgL.size(), R, T, flag);

    // std::vector<cv::Mat> rvecs, tvecs;
    // cv::fisheye::calibrate(real_points, all_cornersL, imgL.size(), cameraMatrixL, distCoeffsL, rvecs, tvecs, flag);
	


    // std::cout << cameraMatrixL << std::endl;
    // std::cout << distCoeffsL << std::endl;
    // std::cout << cameraMatrixR << std::endl;
    // std::cout << R << std::endl;
    // std::cout << T << std::endl;

    cv::Mat RL, RR, PL, PR, Q;
    cv::Mat map1, map2;
    cv::Size newImageSize = cv::Size(imgL.cols, imgL.rows);

    cv::Mat undistortedL, undistortedR;
    imgL = cv::imread("/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_ik/calib_imgs/imageL.png");
    imgR = cv::imread("/home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_ik/calib_imgs/imageR.png");


    cv::fisheye::stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imgL.size(), R, T, RL, RR, PL, PR, Q, 0, newImageSize);

    cv::FileStorage matrixFile("matrices.yml", cv::FileStorage::WRITE);
    matrixFile << "cameraMatrixL" << cameraMatrixL;
    matrixFile << "distCoeffsL" << distCoeffsL;
    matrixFile << "RL" << RL;
    matrixFile << "cameraMatrixR" << cameraMatrixR;
    matrixFile << "distCoeffsR" << distCoeffsR;
    matrixFile << "RR" << RR;

    // cv::fisheye::initUndistortRectifyMap(cameraMatrixL, distCoeffsL, RL, PL, imgL.size(), CV_16SC2, map1, map2);

    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrixL, distCoeffsL, imgL.size(), RL, PL);

    // std::cout << RL << std::endl;
    // std::cout << PL << std::endl;

    cv::Mat newMatrixL = cameraMatrixL.clone();
    newMatrixL.at<double>(0,0) *= 0.6; 
    newMatrixL.at<double>(1,1) *= 0.6; 

    cv::Mat newMatrixR = cameraMatrixR.clone();
    newMatrixR.at<double>(0,0) *= 0.6; 
    newMatrixR.at<double>(1,1) *= 0.6; 

    // std::cout << newMatrixL << std::endl;


    cv::Mat I = cv::Mat::eye(3, 3, cv::DataType<double>::type);

    cv::fisheye::initUndistortRectifyMap(cameraMatrixL, distCoeffsL, I, newMatrixL, imgL.size(), CV_16SC2, map1, map2);
    cv::remap(imgL, undistortedL, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::fisheye::initUndistortRectifyMap(cameraMatrixR, distCoeffsR, RR, newMatrixR, imgR.size(), CV_16SC2, map1, map2);
    cv::remap(imgR, undistortedR, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    
    
    // //cv::fisheye::undistortImage(imgL, undistortedL, cameraMatrixL, distCoeffsL);

    // // std::cout << undistortedL << std::endl;
    cv::imwrite("undistortedL.png", undistortedL);
    cv::imwrite("undistortedR.png", undistortedR);



    return 0;
}
