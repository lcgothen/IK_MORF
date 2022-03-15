#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

#include "coordinates.hpp"
using namespace coords;

#include "controller.hpp"
using namespace controller;

void robot::infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

void angles::calcIK(point target) // calculate angles with IK equations
{
    float xc1, yc1, zc1;
    float L0 = 0.017011, L1 = 0.039805, L2 = 0.070075, L3=0.11542;// L3 = 0.11243;
    float offset2 = 0.8, offset3 = -2.5;

    th1 = atan2(-target.x, target.y);

    xc1 = cos(th1)*target.x+sin(th1)*target.y;
    yc1 = -sin(th1)*target.x+cos(th1)*target.y-L1;
    zc1 = target.z+L0;

    // th2 = acos((pow(L3,2)-pow(yc1,2)-pow(zc1,2)-pow(L2,2))/(-2*L2*sqrt(pow(yc1,2)+pow(zc1,2))))+atan2(zc1,yc1)+offset2;
    // th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3))+offset3; 

    th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
    th2 = atan2(zc1,yc1)+atan2(L3*sin(th3),L2+L3*cos(th3))+offset2;
    th3 += offset3;
}

void CPG::cyclic()
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

    oH1 = outputH1*0.3;
    oH2 = outputH2*0.3;
}

void CPG::walk(images stereo)
{
    float offset2 = 1.5, offset3 = -2.25/4;
    float d=-stereo.target.x;//-0.03-stereo.target.x;

    // front left
    FL.th1 = oH1*(1+d);
    FL.th2 = -oH2+offset2;
    FL.th3 = offset3;
 
    // middle left
    ML.th1 = -oH1*(1+d);
    ML.th2 = oH2+offset2;
    ML.th3 = offset3;

    // back left
    BL.th1 = oH1*(1+d);
    BL.th2 = -oH2+offset2;
    BL.th3 = offset3;

    // front right
    FR.th1 = oH1*(1-d);
    FR.th2 = oH2+offset2;
    FR.th3 = offset3;

    // middle right
    MR.th1 = -oH1*(1-d);
    MR.th2 = -oH2+offset2;
    MR.th3 = offset3;

    // back right
    BR.th1 = oH1*(1-d); 
    BR.th2 = oH2+offset2; 
    BR.th3 = offset3; 

    if(stereo.target.z < 0.1)
        stabilize=true;
}


void images::imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageL = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void images::imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageR = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void images::match()
{
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
                best_matches.push_back(matches[i][0]);
            }
        }
        
        cv::Mat img_matches;
        drawMatches( imageL, keypointsL, imageR, keypointsR, best_matches, img_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        
        imshow("Matches", img_matches);

        cv::Mat output;
        cv::drawKeypoints(imageL, keypointsL, output);
        cv::imshow("viewL", output);
        cv::drawKeypoints(imageR, keypointsR, output);
        cv::imshow("viewR", output);
        cv::waitKey(30);

        float avg_x=0, avg_y=0, avg_z=0, width;

        for (int i = 0; i < best_matches.size(); i++)
        {
            float z = 125*0.06*1/(keypointsL[best_matches[i].queryIdx].pt.x-keypointsR[best_matches[i].trainIdx].pt.x);
            // std::cout << z << std::endl;
            avg_x += keypointsL[best_matches[i].queryIdx].pt.x;
            avg_y += keypointsL[best_matches[i].queryIdx].pt.y;
            avg_z += z;
        }

        avg_x = avg_x/best_matches.size();
        avg_y = avg_y/best_matches.size();
        avg_z = avg_z/best_matches.size();

        width = 2*avg_z*tan(M_PI/4);

        float x,y;

        x = -avg_x/256*width+width/2;
        y = -avg_y/256*width+width/2;

        if(!isnan(x) && !isnan(y) && !isnan(avg_z))
        {
            target.x = x;
            target.y = y;
            target.z = avg_z;
            std::cout << target.x << " , " << target.y <<  " , " << target.z << std::endl;
        }

    }

}

float images::calcHue(float b, float g, float r)
{
    b = b/255;
    g = g/255;
    r = r/255;

    float vmax = std::max(r, std::max(g, b));
    float vmin = std::min(r, std::min(g, b));
    float diff = vmax - vmin;

    float h;

    if (vmax == vmin)
        h = 0;
    else if (vmax == r)
        h = 60 * ((g - b)/(vmax - vmin));
    else if (vmax == g)
        h = 60 * (2 + (b - r)/(vmax - vmin));
    else if (vmax == b)
        h = 60 * (4 + (r - g)/(vmax - vmin));

    if(h<0)
        h += 360;

    return h;
}

void images::blob()
{
    cv::Mat hsvL;
    cv::cvtColor(imageL, hsvL, cv::COLOR_BGR2HSV);
    // cv::inRange(hsvL, cv::Scalar(36, 0, 0), cv::Scalar(86, 255, 255), filteredL);
    // cv::imshow("viewL", imageL);
    //cv::imshow("hsvL", hsvL);

    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 0;


    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypointsL, keypointsR;
    detector->detect(imageL, keypointsL);
    detector->detect(imageR, keypointsR);

    // cv::Mat output;
    // cv::drawKeypoints(imageL, keypointsL, output);
    // cv::imshow("viewL", output);
    // cv::drawKeypoints(imageR, keypointsR, output);
    // cv::imshow("viewR", output);

    int closestGreen=0;
    float hClosest=0;
    for(int i=0; i<keypointsL.size(); i++)
    {
        cv::Vec3b bgrVal = imageL.at<cv::Vec3b>(keypointsL[i].pt);
        cv::Vec3b bgrClosest = imageL.at<cv::Vec3b>(keypointsL[closestGreen].pt);
        float h = calcHue(bgrVal[0], bgrVal[1], bgrVal[2]);

        if(abs(h-140)<abs(hClosest-140))
        {
            closestGreen=i;
            hClosest=h;
        }
    }

    cv::Mat correct;
    std::vector<cv::KeyPoint> correctKeypoint = {keypointsL[closestGreen]};
    cv::drawKeypoints(imageL, correctKeypoint, correct);
    cv::imshow("correct", correct);
    cv::waitKey(30);
}