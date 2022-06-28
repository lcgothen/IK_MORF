#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <string>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "floatfann.h"
#include "fann_cpp.h"

#include "coordinates_real.hpp"
using namespace coords;

#include "controller_real.hpp"
using namespace controller;

// limiter function from /gorobots/projects/C-CPGRBFN/CPGRBFN_BBO_v5/neural_controllers/morf/real/neutronController.cpp
float controller::jointLimiter(float jointValue, float jointMin, float jointMax) 
{
    float toRad = M_PI / 180;
    jointMin = jointMin * toRad;
    jointMax = jointMax * toRad;

    if(jointValue > jointMax)
        return jointMax;
    else if(jointValue < jointMin)
        return jointMin;
    else return jointValue;
}

void robot::jointPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    FL.th1 = msg->data[0];
    FL.th2 = msg->data[1];
    FL.th3 = msg->data[2];
    ML.th1 = msg->data[3];
    ML.th2 = msg->data[4];
    ML.th3 = msg->data[5];
    BL.th1 = msg->data[6];
    BL.th2 = msg->data[7];
    BL.th3 = msg->data[8];
    FR.th1 = msg->data[9];
    FR.th2 = msg->data[10];
    FR.th3 = msg->data[11];
    MR.th1 = msg->data[12];
    MR.th2 = msg->data[13];
    MR.th3 = msg->data[14];
    BR.th1 = msg->data[15];
    BR.th2 = msg->data[16];
    BR.th3 = msg->data[17];
}

void robot::forceSensCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    sensFL = msg->data[2];
}

void robot::foot2buttonPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    foot2button.x = msg->data[0];
    foot2button.y = msg->data[1];
    foot2button.z = msg->data[2];
}

void angles::calcIK(point target) // calculate angles with IK equations
{
    float xc1, yc1, zc1;
    float L0 = 0.017011, L1 = 0.039805, L2 = 0.070075, L3=0.11542;// L3 = 0.11243;
    float offset2 = 0.8, offset3 = -2.5;

    float aux_th1, aux_th2, aux_th3;

    aux_th1 = atan2(-target.x, target.y);

    xc1 = cos(aux_th1)*target.x+sin(aux_th1)*target.y;
    yc1 = -sin(aux_th1)*target.x+cos(aux_th1)*target.y-L1;
    zc1 = target.z+L0;

    // th2 = acos((pow(L3,2)-pow(yc1,2)-pow(zc1,2)-pow(L2,2))/(-2*L2*sqrt(pow(yc1,2)+pow(zc1,2))))+atan2(zc1,yc1)+offset2;
    // th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3))+offset3; 

    aux_th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
    aux_th2 = atan2(zc1,yc1)+atan2(L3*sin(aux_th3),L2+L3*cos(aux_th3))+offset2;
    aux_th3 += offset3;

    // std::cout << target.x << " , " << target.y << " , " << target.z << std::endl;

    if(!isnan(aux_th1) && !isnan(aux_th2) && !isnan(aux_th3))
    {
        th1=aux_th1;
        th2=aux_th2;
        th3=aux_th3;
        
        // std::cout << th1 << " , " << th2 <<  " , " << th3 << std::endl;
    }
    else
    {
        th1=-1.33956;
        th2=2.41989;
        th3=-0.5121;

        std::cout << "at least one invalid angle" << std::endl;
    }
}

void angles::initNN(std::string ann_path)
{
    ann = new struct fann ***[div];

    for(int i=0; i<div; i++)
    {
        ann[i] = new struct fann **[div];
        
        for(int j=0; j<div; j++)
        {
            ann[i][j] = new struct fann *[divZ+divZ_start];

            for(int k=0; k<divZ+divZ_start; k++)
            {
                std::string filename = ann_path+std::to_string(i)+std::to_string(j)+std::to_string(k)+std::string(".net");

                if (FILE *file = fopen(filename.c_str(), "r")) 
                {
                    fclose(file);
                    ann[i][j][k] = fann_create_from_file(filename.c_str());
                }
            }
        }
    }
}

void angles::calcNN(point target)
{
    cubeX=-1; 
    cubeY=-1; 
    cubeZ=-1;

    float x_step = x_length/div;
    float y_step = y_length/div;
    float z_step = z_length/(divZ+divZ_start);


    if(reverse==1)
    {
        if(target.x > x_start)
            cubeX=0;
        else if(target.x < x_start+x_length)
            cubeX=div;
        else 
        {
            float aux = x_start;
            for(int j=0; j<div; j++)
            {
                if(target.x < aux && target.x > aux+x_step)
                {
                    cubeX=j;
                    break;
                }

                aux += x_step;
            }
        }
    }
    else
    {
        if(target.x < x_start)
            cubeX=0;
        else if(target.x > x_start+x_length)
            cubeX=div;
        else 
        {
            float aux = x_start;
            for(int j=0; j<div; j++)
            {
                if(target.x > aux && target.x < aux+x_step)
                {
                    cubeX=j;
                    break;
                }

                aux += x_step;
            }
        }
    }

    if(target.y < y_start)
        cubeY=0;
    else if(target.y > y_start+y_length)
        cubeY=div;
    else 
    {
        float aux = y_start;
        for(int k=0; k<div; k++)
        {
            if(target.y > aux && target.y < aux+y_step)
            {
                cubeY=k;
                break;
            }

            aux += y_step;
        }
    }

    if(target.z < z_start + z_step*divZ_start)
        cubeZ=divZ_start;
    else if(target.z > z_start+z_length)
        cubeZ=divZ+divZ_start;
    else 
    {
        float aux = z_start + z_step*divZ_start; // downsizing in z for babbling data
        for(int l=divZ_start; l<divZ+divZ_start; l++) 
        {
            if(target.z > aux && target.z < aux+z_step)
            {
                cubeZ=l;
                break;
            }

            aux += z_step;
        }
    }

    // std::cout << div << " , " << divZ << std::endl;

    // std::cout << cubeX << " , " << cubeY << " , " << cubeZ << std::endl;
    // std::cout << target.x << " , " << target.y <<  " , " << target.z << std::endl;

    if(cubeX!=-1 && cubeY!=-1 && cubeZ!=-1)
    {
        point legTarget = target; // (target.*morf2leg)();

        // std::string filename = ann_path+std::to_string(cubeX)+std::to_string(cubeY)+std::to_string(cubeZ)+std::string(".net");
        // struct fann *ann = fann_create_from_file(filename.c_str());
        float input[3] = {legTarget.x, legTarget.y, legTarget.z};
        fann_scale_input(ann[cubeX][cubeY][cubeZ], input);
        float *output = fann_run(ann[cubeX][cubeY][cubeZ], input);
        fann_descale_output(ann[cubeX][cubeY][cubeZ], output);

        th1 = output[0];
        th2 = output[1];
        th3 = output[2];

    }
    
    // std::cout << cubeX << " , " << cubeY << " , " << cubeZ << std::endl;
}

void CPG::cyclic(images *stereo)
{
    float W12 = 0.4; 
    float W21 = -0.4; 
    float W11 = 1.5; 
    float W22 = 1.5;
    float BiasH1 = 0.01;
    float BiasH2 = 0.01;

    float activityH1 = W11*outputH1 + W12*outputH2 + BiasH1;
    float activityH2 = W22*outputH2 + W21*outputH1 + BiasH2;

    outputH1 = tanh(activityH1);
    outputH2 = tanh(activityH2);

    if(stereo->target.z >= stereo->threshZ)
        stereo->distZ=0;
    else
        stereo->distZ++;

    if(stereo->distZ>=2)
    {
        if(stereo->threshZ<=0.15)
            stereo->nearZ=true;

        k-=0.04;
        stereo->threshZ-=0.155;
        stereo->distZ=0;

        // std::cout << k << " , " << stereo->threshZ << " , " << std::boolalpha << stereo->nearZ << std::endl;

    }

    oH1 = outputH1*k;
    oH2 = outputH2*0.3;
}

void CPG::walk(images stereo)
{
    float offset2 = 1.5, offset3 = -2.25/3.5;
    float d=0.04-stereo.target.x;

    // joint limits from /gorobots/projects/C-CPGRBFN/CPGRBFN_BBO_v5/neural_controllers/morf/real/neutronController.cpp
    // front left
    FL.th1 = jointLimiter(oH1*(1+d), -17, 17);
    FL.th2 = jointLimiter(-oH2+offset2, -5, 130);
    FL.th3 = offset3;
 
    // middle left
    ML.th1 = jointLimiter(-oH1*(1+d), -17, 17);
    ML.th2 = jointLimiter(oH2+offset2, -5, 130);
    ML.th3 = offset3;

    // back left
    BL.th1 = jointLimiter(oH1*(1+d), -17, 17);
    BL.th2 = jointLimiter(-oH2+offset2, -5, 130);
    BL.th3 = offset3;

    // right side has the th1 angles inverted because of the orientation of the legs on the robot
    // front right
    FR.th1 = jointLimiter(oH1*(1-d), -17, 17);
    FR.th2 = jointLimiter(oH2+offset2, -5, 130);
    FR.th3 = offset3;

    // middle right
    MR.th1 = jointLimiter(-oH1*(1-d), -17, 17);
    MR.th2 = jointLimiter(-oH2+offset2, -5, 130);
    MR.th3 = offset3;

    // back right
    BR.th1 = jointLimiter(oH1*(1-d), -17, 17);
    BR.th2 = jointLimiter(oH2+offset2, -5, 130);
    BR.th3 = offset3; 
}

void CPG::simple_cyclic()
{
    float W12 = 0.4; 
    float W21 = -0.4; 
    float W11 = 1.5; 
    float W22 = 1.5;
    float BiasH1 = 0.01;
    float BiasH2 = 0.01;

    float activityH1 = W11*outputH1 + W12*outputH2 + BiasH1;
    float activityH2 = W22*outputH2 + W21*outputH1 + BiasH2;

    outputH1 = tanh(activityH1);
    outputH2 = tanh(activityH2);

    oH1 = outputH1*k;
    oH2 = outputH2*0.3;
}

void CPG::gait(int type)
{
    float offset2 = 1.5, offset3 = -2.25/3.5;

    // joint limits from /gorobots/projects/C-CPGRBFN/CPGRBFN_BBO_v5/neural_controllers/morf/real/neutronController.cpp

    if(type==1)
    {
        // front left
        FL.th1 = jointLimiter(oH1, -17, 17);
        FL.th2 = jointLimiter(-oH2+offset2, -5, 130);
        FL.th3 = offset3;
    
        // middle left
        ML.th1 = jointLimiter(-oH1, -17, 17);
        ML.th2 = jointLimiter(oH2+offset2, -5, 130);
        ML.th3 = offset3;

        // back left
        BL.th1 = jointLimiter(oH1, -17, 17);
        BL.th2 = jointLimiter(-oH2+offset2, -5, 130);
        BL.th3 = offset3;

        // right side has the th1 angles inverted because of the orientation of the legs on the robot
        // front right
        FR.th1 = jointLimiter(oH1, -17, 17);
        FR.th2 = jointLimiter(oH2+offset2, -5, 130);
        FR.th3 = offset3;

        // middle right
        MR.th1 = jointLimiter(-oH1, -17, 17);
        MR.th2 = jointLimiter(-oH2+offset2, -5, 130);
        MR.th3 = offset3;

        // back right
        BR.th1 = jointLimiter(oH1, -17, 17);
        BR.th2 = jointLimiter(oH2+offset2, -5, 130);
        BR.th3 = offset3; 
    }
}

void images::imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageL = cv_bridge::toCvShare(msg, "bgr8")->image;
    // imwrite("imageL.png", imageL);
    imwrite("fisheyeL.png", imageL);
}

void images::imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageR = cv_bridge::toCvShare(msg, "bgr8")->image;
    // imwrite("imageR.png", imageR);
    imwrite("fisheyeR.png", imageR);
}

void images::generalImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    genImg = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void images::blob()
{
    cv::Mat undistortedL, undistortedR;

    cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, RL, RR, newMatrixL, newMatrixR;
    cv::FileStorage matrixFile("matrices.yml", cv::FileStorage::READ);
    matrixFile["cameraMatrixL"] >> cameraMatrixL;
    matrixFile["distCoeffsL"] >> distCoeffsL;
    matrixFile["cameraMatrixR"] >> cameraMatrixR;
    matrixFile["distCoeffsR"] >> distCoeffsR;
    matrixFile["RL"] >> RL;
    matrixFile["RR"] >> RR;
    matrixFile["newMatrixL"] >> newMatrixL;
    matrixFile["newMatrixR"] >> newMatrixR;

    cv::Mat map1, map2;
    cv::Mat I = cv::Mat::eye(3, 3, cv::DataType<double>::type);

    // std::cout << cameraMatrixL << std::endl;

    cv::fisheye::initUndistortRectifyMap(cameraMatrixL, distCoeffsL, I, newMatrixL, imageL.size(), CV_16SC2, map1, map2);
    cv::remap(imageL, imageL, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::fisheye::initUndistortRectifyMap(cameraMatrixR, distCoeffsR, I, newMatrixR, imageR.size(), CV_16SC2, map1, map2);
    cv::remap(imageR, imageR, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 100;
    params.filterByCircularity = false;
    params.minCircularity = 0.9;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.6;



    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypointsL, keypointsR;
    detector->detect(imageL, keypointsL);
    detector->detect(imageR, keypointsR);

    cv::Mat imgKey;
    
    cv::drawKeypoints(imageL, keypointsL, imgKey, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imwrite("keypointL.png", imgKey);
    imwrite("imageL.png", imageL);
    cv::drawKeypoints(imageR, keypointsR, imgKey, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imwrite("keypointR.png", imgKey);
    imwrite("imageR.png", imageR);


    if(!keypointsL.empty() && !keypointsR.empty())
    {

        float f=newMatrixL.at<double>(0,0); //focal_length
        // std::cout << f << std::endl;

        target.z = f*0.07/(keypointsL[0].pt.x-keypointsR[0].pt.x);

        float fov_x = atan2(newMatrixL.at<double>(0,2), f);
        float fov_y = atan2(newMatrixL.at<double>(1,2), f);

        float width = 848/newMatrixL.at<double>(0,2)*target.z*tan(fov_x);
        float height = 800/(800-newMatrixL.at<double>(1,2))*target.z*tan(fov_y);

        target.x = -keypointsL[0].pt.x/848*width + newMatrixL.at<double>(0,2)/848*width; 
        target.y = -keypointsL[0].pt.y/800*height + (800-newMatrixL.at<double>(1,2))/800*height;

        std::cout << target.x << " , " << target.y << " , " << target.z << std::endl;
    }
}