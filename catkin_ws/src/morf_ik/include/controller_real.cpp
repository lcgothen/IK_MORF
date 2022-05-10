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

#include "floatfann.h"
#include "fann_cpp.h"

#include "coordinates.hpp"
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

    if(!isnan(aux_th1) && !isnan(aux_th2) && !isnan(aux_th3))
    {
        th1=aux_th1;
        th2=aux_th2;
        th3=aux_th3;
    }
    else
    {
        th1=-1.33956;
        th2=2.41989;
        th3=-0.5121;
    }
}

// void angles::calcNN(point target, coords::point (coords::point::*morf2leg)(), std::string ann_path)
// {
//     int cubeX=-1, cubeY=-1, cubeZ=-1;

//     // smaller - morf
//     // float x_length = 0.19010;
//     // float y_length = 0.26011;
//     // float z_length = 0.21494;

//     // float x_start = -7.4826e-02;
//     // float y_start = +6.2794e-02;
//     // float z_start = -3.8406e-01;

//     // bigger - morf
//     // float x_length = 0.23010;
//     // float y_length = 0.30011;
//     // float z_length = 0.25494;

//     // float x_start = -5.4826e-02;
//     // float y_start = +8.2794e-02;
//     // float z_start = -3.6406e-01;

//     // bigger - FL
//     float x_length = -0.25494;
//     float y_length = 0.30011;
//     float z_length = 0.2301;

//     float x_start = 0.225164; 
//     float y_start = -0.057090; 
//     float z_start = -0.0433698; 

//     int div=4;

//     float x_step = x_length/div;
//     float y_step = y_length/div;
//     float z_step = z_length/div;

//     for(int j=0; j<div; j++)
//     {
//         if(target.x < x_start && target.x > x_start+x_step)
//         {
//             cubeX=j;
//             break;
//         }

//         x_start += x_step;
//     }

//     for(int k=0; k<div; k++)
//     {
//         if(target.y > y_start && target.y < y_start+y_step)
//         {
//             cubeY=k;
//             break;
//         }

//         y_start += y_step;
//     }

//     for(int l=0; l<div; l++)
//     {
//         if(target.z > z_start && target.z < z_start+z_step)
//         {
//             cubeZ=l;
//             break;
//         }

//         z_start += z_step;
//     }

//     if(cubeX!=-1 && cubeY!=-1 && cubeZ!=-1)
//     {
//         point legTarget = target; // (target.*morf2leg)();

//         std::string filename = ann_path+std::to_string(cubeX)+std::to_string(cubeY)+std::to_string(cubeZ)+std::string(".net");
//         struct fann *ann = fann_create_from_file(filename.c_str());
//         float input[3] = {legTarget.x, legTarget.y, legTarget.z};
//         fann_scale_input(ann, input);
//         float *output = fann_run(ann, input);
//         fann_descale_output(ann, output);

//         th1 = output[0];
//         th2 = output[1];
//         th3 = output[2];

//     }
    
//     // std::cout << target.x << " , " << target.y <<  " , " << target.z << std::endl;
//     // std::cout << cubeX << " , " << cubeY << " , " << cubeZ << std::endl;
// }

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

    if(stereo->distZ>=5)
    {
        if(stereo->threshZ<=0.2)
            stereo->nearZ=true;

        k-=0.05;
        stereo->threshZ-=0.15;
        stereo->distZ=0;

        // std::cout << k << " , " << stereo->threshZ << " , " << std::boolalpha << stereo->nearZ << std::endl;

    }

    // if(stereo.target.z < 0.2 && stereo.target.z >= 0 && k>0.2)
    //     k=0.06;
    // else if(stereo.target.z < 0.3 && stereo.target.z >= 0 && k>0.4)
    //     k=0.12;
    // else if(stereo.target.z < 0.4 && stereo.target.z >= 0 && k>0.6)
    //     k=0.18;
    // else if(stereo.target.z < 0.5 && stereo.target.z >= 0 && k>0.8)
    //     k=0.24;

    oH1 = outputH1*k;
    oH2 = outputH2*k;
}

void CPG::walk(images stereo)
{
    float offset2 = 1.5, offset3 = -2.25/4;
    float d=0.07-stereo.target.x;//0.08-1.2*stereo.target.x;//-0.03-stereo.target.x;

    // joint limits from /gorobots/projects/C-CPGRBFN/CPGRBFN_BBO_v5/neural_controllers/morf/real/neutronController.cpp
    // front left
    FL.th1 = jointLimiter(oH1*(1+d), -110, 22);
    FL.th2 = jointLimiter(-oH2+offset2, -5, 130);
    FL.th3 = offset3;
 
    // middle left
    ML.th1 = jointLimiter(-oH1*(1+d), -22, 22);
    ML.th2 = jointLimiter(oH2+offset2, -5, 130);
    ML.th3 = offset3;

    // back left
    BL.th1 = jointLimiter(oH1*(1+d), -22, 110);
    BL.th2 = jointLimiter(-oH2+offset2, -5, 130);
    BL.th3 = offset3;

    // right side has the th1 angles inverted because of the orientation of the legs on the robot
    // front right
    FR.th1 = jointLimiter(oH1*(1-d), -22, 110);
    FR.th2 = jointLimiter(oH2+offset2, -5, 130);
    FR.th3 = offset3;

    // middle right
    MR.th1 = jointLimiter(-oH1*(1-d), -22, 22);
    MR.th2 = jointLimiter(-oH2+offset2, -5, 130);
    MR.th3 = offset3;

    // back right
    BR.th1 = jointLimiter(oH1*(1-d), -110, 22);
    BR.th2 = jointLimiter(oH2+offset2, -5, 130);
    BR.th3 = offset3; 
}

void images::imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat gray = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::applyColorMap(gray, imageL, cv::COLORMAP_JET);
    
    imwrite("imageL.png", imageL);
}

void images::imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageR = cv_bridge::toCvShare(msg, "bgr8")->image;
    imwrite("imageR.png", imageR);
}

void images::generalImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    genImg = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void images::match()
{
    cv::Mat hsvL, hsvR;
    cv::cvtColor(imageL, hsvL, cv::COLOR_BGR2HSV);
    cv::cvtColor(imageR, hsvR, cv::COLOR_BGR2HSV);

    // detect features	
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> keypointsL, keypointsR;
    detector->detect(imageL, keypointsL);
    detector->detect(imageR, keypointsR);
    
    // extract descriptors
    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
    cv::Mat descriptorsL, descriptorsR;
    extractor->compute(imageL, keypointsL, descriptorsL);
    extractor->compute(imageR, keypointsR, descriptorsR);


    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));

    std::vector< std::vector<cv::DMatch> > matches;
    if(!descriptorsL.empty() && !descriptorsR.empty())
    {
        matcher.knnMatch(descriptorsL, descriptorsR, matches, 2);
        
        const float ratio_thresh = 0.6f;
        std::vector<cv::DMatch> best_matches;
        
        for (int i = 0; i < matches.size(); i++)
        {
            // Only save matches according to Lowe's ratio test
            if(!matches[i].empty() && matches[i][0].distance < ratio_thresh * matches[i][1].distance)
            {
                cv::Vec3b hsvValL = hsvL.at<cv::Vec3b>(keypointsL[matches[i][0].queryIdx].pt);
                float hL = hsvValL[0];
                cv::Vec3b hsvValR = hsvR.at<cv::Vec3b>(keypointsR[matches[i][0].trainIdx].pt);
                float hR = hsvValR[0];

                if(hL>40 && hL<80 && hR>40 && hR<80)
                {
                    best_matches.push_back(matches[i][0]);
                }
            }
        }
        
        cv::Mat img_matches;
        drawMatches( imageL, keypointsL, imageR, keypointsR, best_matches, img_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        
        imshow("Matches", img_matches);
        cv::waitKey(30);

        float avg_x=0, avg_y=0, avg_z=0;

        for (int i = 0; i < best_matches.size(); i++)
        {
            float z = 175.05*0.06*1/(keypointsL[best_matches[i].queryIdx].pt.x-keypointsR[best_matches[i].trainIdx].pt.x);
            // std::cout << z << std::endl;
            avg_x += keypointsL[best_matches[i].queryIdx].pt.x;
            avg_y += keypointsL[best_matches[i].queryIdx].pt.y;
            avg_z += z;
        }

        avg_x = avg_x/best_matches.size();
        avg_y = avg_y/best_matches.size();
        avg_z = avg_z/best_matches.size();

        float length = 2*avg_z*tan(2.3562/2);
        float width = 800*length/848;

        float x,y;
        x = -avg_x/848*length+length/2;
        y = -avg_y/800*width+width/2;

        if(!isnan(x) && !isnan(y) && !isnan(avg_z))
        {
            target.x = x;
            target.y = y;
            target.z = avg_z;

            target_avg.z = target.z;
            target_avg.x = target.x;
            target_avg.y = target.y;
            // std::cout << target.x << " , " << target.y <<  " , " << target.z << std::endl;
        }

    }

}

void images::blob()
{
    cv::Mat hsvL, hsvR;
    cv::cvtColor(imageL, hsvL, cv::COLOR_BGR2HSV);
    cv::cvtColor(imageR, hsvR, cv::COLOR_BGR2HSV);

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 0;
    // params.maxArea = 100;


    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypointsL, keypointsR, keypointsGreenL, keypointsGreenR;
    detector->detect(imageL, keypointsL);
    detector->detect(imageR, keypointsR);

    int closestGreenL=0, closestGreenR=0;
    float hClosestL=0, hClosestR=0;

    float x_avgL=0, y_avgL=0, x_avgR=0, y_avgR=0, quantL=0, quantR=0;

    for(int i=0; i<keypointsL.size(); i++)
    {
        cv::Vec3b bgrVal = imageL.at<cv::Vec3b>(keypointsL[i].pt);
        cv::Vec3b bgrClosest = imageL.at<cv::Vec3b>(keypointsL[closestGreenL].pt);

        cv::Vec3b hsvVal = hsvL.at<cv::Vec3b>(keypointsL[i].pt);
        float h = hsvVal[0];

        if(abs(h-60)<abs(hClosestL-60))
        {
            closestGreenL=i;
            hClosestL=h;
        }

        if(h>40 && h<80)
        {
            x_avgL+=keypointsL[i].pt.x;
            y_avgL+=keypointsL[i].pt.y;
            quantL++;
            keypointsGreenL.push_back(keypointsL[i]);
        }

        //std::cout << h << std::endl;
    }

    for(int i=0; i<keypointsR.size(); i++)
    {
        cv::Vec3b bgrVal = imageR.at<cv::Vec3b>(keypointsR[i].pt);
        cv::Vec3b bgrClosest = imageR.at<cv::Vec3b>(keypointsR[closestGreenR].pt);


        cv::Vec3b hsvVal = hsvR.at<cv::Vec3b>(keypointsR[i].pt);
        float h = hsvVal[0];

        if(abs(h-60)<abs(hClosestR-60))
        {
            closestGreenR=i;
            hClosestR=h;
        }

        if(h>40 && h<80)
        {
            x_avgR+=keypointsR[i].pt.x;
            y_avgR+=keypointsR[i].pt.y;
            quantR++;
            keypointsGreenR.push_back(keypointsR[i]);
        }
    }


    if(!keypointsL.empty() && !keypointsR.empty())
    {
        cv::Mat correct;
        std::vector<cv::KeyPoint> correctKeypoint = {keypointsL[closestGreenL]};
        cv::drawKeypoints(imageL, keypointsGreenL, correct);
        //cv::drawKeypoints(imageL, keypointsGreenL, correct, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow("correctL", correct);
        correctKeypoint = {keypointsR[closestGreenR]};
        cv::drawKeypoints(imageR, keypointsGreenR, correct);
        cv::imshow("correctR", correct);
        cv::waitKey(30);

        target.z = 175.05*0.06*1/(keypointsL[closestGreenL].pt.x-keypointsR[closestGreenR].pt.x);

        float length = 2*target.z*tan(2.3562/2);
        float width = 800*length/848;

        target.x = -keypointsL[closestGreenL].pt.x/848*length+length/2;
        target.y = -keypointsL[closestGreenL].pt.y/800*width+width/2;

        if(quantL>0)
        {
            x_avgL=x_avgL/quantL;
            y_avgL=y_avgL/quantL;
        }
        if(quantR>0)
        {
            x_avgR=x_avgR/quantR;
            y_avgR=y_avgR/quantR;
        }

        target_avg.z = 175.05*0.06*1/(x_avgL-x_avgR);

        length = 2*target_avg.z*tan(2.3562/2);
        width = 800*length/848;

        target_avg.x = -x_avgL/848*length+length/2;
        target_avg.y = -y_avgL/800*width+width/2;

        
        // std::cout << target_avg.x << " , " << target_avg.y << " , " << target_avg.z << std::endl;
    }
}