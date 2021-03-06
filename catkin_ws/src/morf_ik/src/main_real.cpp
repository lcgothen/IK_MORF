#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <sys/stat.h>
#include <chrono>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"



#include "../include/coordinates_real.hpp"
#include "../include/controller_real.hpp"
using namespace coords;
using namespace controller;

/********************* USAGE *************************

./main TYPE

TYPE_: 0 means equations, 1 means neural network

******************************************************/


int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "./morf_controller_real TYPE" << std::endl;
        exit(EXIT_FAILURE);
    }

    int type = std::stoi(argv[1]);

    std::string ann_path;
    angles FL;

    if(type==1)
    {
        cv::FileStorage configFile("ann_config.yml", cv::FileStorage::READ);
        configFile["ann_path"] >> ann_path;
        configFile["div"] >> FL.div;
        configFile["divZ"] >> FL.divZ;
        configFile["divZ_start"] >> FL.divZ_start;
        configFile["x_length"] >> FL.x_length;
        configFile["y_length"] >> FL.y_length;
        configFile["z_length"] >> FL.z_length;
        configFile["x_start"] >> FL.x_start;
        configFile["y_start"] >> FL.y_start;
        configFile["z_start"] >> FL.z_start;
        configFile["reverse"] >> FL.reverse;

        FL.initNN(ann_path);
    }

    point posFL;

    angles FR, ML, MR, BL, BR;
    angles auxFL, auxML, auxBL, auxFR, auxMR, auxBR;


    angles default_;

    default_.th1=0;
    default_.th2=2.024319;
    default_.th3=0;


    FL.th1=-1.33956;
    FL.th2=1.6478;
    FL.th3=-0.606795;

    ML.th1=-1.10023;
    ML.th2=1.40566;
    ML.th3=-1.03788;

    BL.th1=-0.507609;
    BL.th2=2.00719;
    BL.th3=0.0646126;

    FR.th1=1.33837;
    FR.th2=1.65268;
    FR.th3=-0.59803;

    MR.th1=1.10005;
    MR.th2=1.40621;
    MR.th3=-1.03692;

    BR.th1=0.506971;
    BR.th2=2.00742;
    BR.th3=0.0650275;


    auxFL.th1=-1.338374;
    auxFL.th2=2.429317;
    auxFL.th3=-0.503011;

    auxML.th1=-1.100229;
    auxML.th2=2.324319;
    auxML.th3=-0.2;

    auxBL.th1=-0.507609;
    auxBL.th2=2.224319;
    auxBL.th3=0;

    auxFR.th1=1.338374;
    auxFR.th2=2.429317;
    auxFR.th3=-0.503011;

    auxMR.th1=1.100229;
    auxMR.th2=2.324319;
    auxMR.th3=-0.2;

    auxBR.th1=0.507609;
    auxBR.th2=2.224319;
    auxBR.th3=0;


    robot morf;
    images stereo;
    CPG cpg;
    bool stable=false, inPos=false, done=false;
    int state=0, stable_state=0;
    point target;

    ros::init(argc, argv, "IK_controller");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("/morf_hw/multi_joint_command", 1000);

    ros::Subscriber jointPos_sub = n.subscribe("/morf_hw/joint_positions", 1000, &robot::jointPosCallback, &morf);
    ros::Subscriber forceSens_sub = n.subscribe("/morf_hw/joint_torques", 1000, &robot::forceSensCallback, &morf);

    image_transport::Subscriber imageLeft_sub = it.subscribe("/camera/fisheye1/image_raw", 1, &images::imageLeftCallback, &stereo);
    image_transport::Subscriber imageRight_sub = it.subscribe("/camera/fisheye2/image_raw", 1, &images::imageRightCallback, &stereo);


    ros::Rate loop_rate(10);
    std_msgs::Float32MultiArray IK_order;

    while(ros::ok())
    {
        /********************** VISION AND IK TEST **********************/
        //std::cout << stereo.nearZ << std::endl;
        // if(!stereo.imageL.empty() && !stereo.imageR.empty() && !stereo.nearZ && !done)
        // {
        //     stereo.blob();
        // }
        // else if(stereo.nearZ && !done)
        // {
        //     posFL = stereo.target.camLeft2morf(); 
        //     posFL = posFL.morf2FL(); 
        //     FL.calcIK(posFL);

        //     IK_order.data.clear();
        //     IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
        //                         21, ML.th1, 22, ML.th2, 23, ML.th3,
        //                         31, BL.th1, 32, BL.th2, 33, BL.th3,
        //                         41, FR.th1, 42, FR.th2, 43, FR.th3,
        //                         51, MR.th1, 52, MR.th2, 53, MR.th3,
        //                         61, BR.th1, 62, BR.th2, 63, BR.th3};
        //     controller_pub.publish(IK_order);
        //     done = true;
        // }
        /********************** VISION AND IK TEST **********************/


        /********************** CPG TEST **********************/
        // cpg.cyclic(&stereo);
        // cpg.walk(stereo);
        // IK_order.data.clear();
        // IK_order.data =    {11, cpg.FL.th1, 12, cpg.FL.th2, 13, cpg.FL.th3,
        //                     21, cpg.ML.th1, 22, cpg.ML.th2, 23, cpg.ML.th3,
        //                     31, cpg.BL.th1, 32, cpg.BL.th2, 33, cpg.BL.th3,
        //                     41, cpg.FR.th1, 42, cpg.FR.th2, 43, cpg.FR.th3,
        //                     51, cpg.MR.th1, 52, cpg.MR.th2, 53, cpg.MR.th3,
        //                     61, cpg.BR.th1, 62, cpg.BR.th2, 63, cpg.BR.th3};
        // controller_pub.publish(IK_order);
        /********************** CPG TEST **********************/

        float distance;
        float but_rad = 0.02;
        bool distFail = false, durFail = false;
        float margin = 0.1;

        std::chrono::microseconds duration=std::chrono::microseconds(0);
        std::chrono::microseconds trial_dur;
        float num_calcs=0;
        typedef std::chrono::high_resolution_clock Clock;
        typedef std::chrono::microseconds microseconds;
        Clock::time_point init = Clock::now();
        Clock::time_point init_calc;

        if(!stereo.imageL.empty() && !stereo.imageR.empty() && !stable)
        {
            stereo.blob();
        }


        if(state==0 && !stereo.imageL.empty() && !stereo.imageR.empty())
            state=1;
        else if(state==1 && stereo.nearZ)
            state=2;
        else if(state==2 && stable)
            state=3;
        else if(state==3 && morf.FL.th1<FL.th1+margin && morf.FL.th1>FL.th1-margin && 
                morf.FL.th2<FL.th2+margin && morf.FL.th2>FL.th2-margin &&
                morf.FL.th3<FL.th3+margin && morf.FL.th3>FL.th3-margin)
            state=4;
        else if(state==4 && morf.sensFL<-0.3)
            state=5;

        if(state==0)
        {
            IK_order.data.clear();
            IK_order.data =    {11, default_.th1, 12, default_.th2, 13, default_.th3,
                                21, default_.th1, 22, default_.th2, 23, default_.th3,
                                31, default_.th1, 32, default_.th2, 33, default_.th3,
                                41, default_.th1, 42, default_.th2, 43, default_.th3,
                                51, default_.th1, 52, default_.th2, 53, default_.th3,
                                61, default_.th1, 62, default_.th2, 63, default_.th3};
            controller_pub.publish(IK_order);
        }
        else if(state==1)
        {
            cpg.cyclic(&stereo);
            cpg.walk(stereo);

            IK_order.data.clear();
            IK_order.data =    {11, cpg.FL.th1, 12, cpg.FL.th2, 13, cpg.FL.th3,
                                21, cpg.ML.th1, 22, cpg.ML.th2, 23, cpg.ML.th3,
                                31, cpg.BL.th1, 32, cpg.BL.th2, 33, cpg.BL.th3,
                                41, cpg.FR.th1, 42, cpg.FR.th2, 43, cpg.FR.th3,
                                51, cpg.MR.th1, 52, cpg.MR.th2, 53, cpg.MR.th3,
                                61, cpg.BR.th1, 62, cpg.BR.th2, 63, cpg.BR.th3};
            controller_pub.publish(IK_order);
        }
        else if(state==2)
        {
            if(stable_state==0 && morf.FL.th1<auxFL.th1+margin && morf.FL.th1>auxFL.th1-margin && 
                    morf.FL.th2<auxFL.th2+margin && morf.FL.th2>auxFL.th2-margin &&
                    morf.FL.th3<auxFL.th3+margin && morf.FL.th3>auxFL.th3-margin)
                stable_state=1;
            else if(stable_state==1 && morf.FL.th1<FL.th1+margin && morf.FL.th1>FL.th1-margin && 
                    morf.FL.th2<FL.th2+margin && morf.FL.th2>FL.th2-margin &&
                    morf.FL.th3<FL.th3+margin && morf.FL.th3>FL.th3-margin)
                stable_state=2;
            else if(stable_state==2 && morf.BL.th1<auxBL.th1+margin && morf.BL.th1>auxBL.th1-margin && 
                    morf.BL.th2<auxBL.th2+margin && morf.BL.th2>auxBL.th2-margin &&
                    morf.BL.th3<auxBL.th3+margin && morf.BL.th3>auxBL.th3-margin)
                stable_state=3;
            else if(stable_state==3 && morf.BL.th1<BL.th1+margin && morf.BL.th1>BL.th1-margin && 
                    morf.BL.th2<BL.th2+margin && morf.BL.th2>BL.th2-margin &&
                    morf.BL.th3<BL.th3+margin && morf.BL.th3>BL.th3-margin)
                stable_state=4;
            else if(stable_state==4 && morf.ML.th1<auxML.th1+margin && morf.ML.th1>auxML.th1-margin && 
                    morf.ML.th2<auxML.th2+margin && morf.ML.th2>auxML.th2-margin &&
                    morf.ML.th3<auxML.th3+margin && morf.ML.th3>auxML.th3-margin)
                stable_state=5;
            else if(stable_state==5 && morf.ML.th1<ML.th1+margin && morf.ML.th1>ML.th1-margin && 
                    morf.ML.th2<ML.th2+margin && morf.ML.th2>ML.th2-margin &&
                    morf.ML.th3<ML.th3+margin && morf.ML.th3>ML.th3-margin)
                stable_state=6;
            else if(stable_state==6 && morf.FR.th1<auxFR.th1+margin && morf.FR.th1>auxFR.th1-margin && 
                    morf.FR.th2<auxFR.th2+margin && morf.FR.th2>auxFR.th2-margin &&
                    morf.FR.th3<auxFR.th3+margin && morf.FR.th3>auxFR.th3-margin)
                stable_state=7;
            else if(stable_state==7 && morf.FR.th1<FR.th1+margin && morf.FR.th1>FR.th1-margin && 
                    morf.FR.th2<FR.th2+margin && morf.FR.th2>FR.th2-margin &&
                    morf.FR.th3<FR.th3+margin && morf.FR.th3>FR.th3-margin)
                stable_state=8;
            else if(stable_state==8 && morf.BR.th1<auxBR.th1+margin && morf.BR.th1>auxBR.th1-margin && 
                    morf.BR.th2<auxBR.th2+margin && morf.BR.th2>auxBR.th2-margin &&
                    morf.BR.th3<auxBR.th3+margin && morf.BR.th3>auxBR.th3-margin)
                stable_state=9;
            else if(stable_state==9 && morf.BR.th1<BR.th1+margin && morf.BR.th1>BR.th1-margin && 
                    morf.BR.th2<BR.th2+margin && morf.BR.th2>BR.th2-margin &&
                    morf.BR.th3<BR.th3+margin && morf.BR.th3>BR.th3-margin)
                stable_state=10;
            else if(stable_state==10 && morf.MR.th1<auxMR.th1+margin && morf.MR.th1>auxMR.th1-margin && 
                    morf.MR.th2<auxMR.th2+margin && morf.MR.th2>auxMR.th2-margin &&
                    morf.MR.th3<auxMR.th3+margin && morf.MR.th3>auxMR.th3-margin)
                stable_state=11;
            else if(stable_state==11 && morf.MR.th1<MR.th1+margin && morf.MR.th1>MR.th1-margin && 
                    morf.MR.th2<MR.th2+margin && morf.MR.th2>MR.th2-margin &&
                    morf.MR.th3<MR.th3+margin && morf.MR.th3>MR.th3-margin)
                stable_state=12;

            IK_order.data.clear();

            if(stable_state==0)
            {
                IK_order.data =    {11, auxFL.th1, 12, auxFL.th2, 13, auxFL.th3,
                                    21, default_.th1, 22, default_.th2, 23, default_.th3,
                                    31, default_.th1, 32, default_.th2, 33, default_.th3,
                                    41, default_.th1, 42, default_.th2, 43, default_.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==1)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, default_.th1, 22, default_.th2, 23, default_.th3,
                                    31, default_.th1, 32, default_.th2, 33, default_.th3,
                                    41, default_.th1, 42, default_.th2, 43, default_.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==2)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, default_.th1, 22, default_.th2, 23, default_.th3,
                                    31, auxBL.th1, 32, auxBL.th2, 33, auxBL.th3,
                                    41, default_.th1, 42, default_.th2, 43, default_.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==3)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, default_.th1, 22, default_.th2, 23, default_.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, default_.th1, 42, default_.th2, 43, default_.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==4)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, auxML.th1, 22, auxML.th2, 23, auxML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, default_.th1, 42, default_.th2, 43, default_.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==5)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, default_.th1, 42, default_.th2, 43, default_.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==6)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, auxFR.th1, 42, auxFR.th2, 43, auxFR.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==7)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, default_.th1, 62, default_.th2, 63, default_.th3};
            }
            else if(stable_state==8)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, auxBR.th1, 62, auxBR.th2, 63, auxBR.th3};
            }
            else if(stable_state==9)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, default_.th1, 52, default_.th2, 53, default_.th3,
                                    61, BR.th1, 62, BR.th2, 63, BR.th3};
            }
            else if(stable_state==10)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, auxMR.th1, 52, auxMR.th2, 53, auxMR.th3,
                                    61, BR.th1, 62, BR.th2, 63, BR.th3};
            }
            else if(stable_state==11)
            {
                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, MR.th1, 52, MR.th2, 53, MR.th3,
                                    61, BR.th1, 62, BR.th2, 63, BR.th3};
            }
            else if(stable_state==12)
            {
                stable=true;

                IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, MR.th1, 52, MR.th2, 53, MR.th3,
                                    61, BR.th1, 62, BR.th2, 63, BR.th3};
            }

            controller_pub.publish(IK_order);
        }
        else if(state==3)
        {
            target.x=stereo.target.x;
            target.y=stereo.target.y;
            target.z=stereo.target.z-0.05;

            posFL = target.camLeft2morf(); 
            posFL = posFL.morf2FL(); 
            if(type==0)
            {
                init_calc = Clock::now();
                FL.calcIK(posFL);
                duration += std::chrono::duration_cast<microseconds>(Clock::now() - init_calc);
                num_calcs++;
            }
            else if(type==1)
            {
                init_calc = Clock::now();
                FL.calcNN(posFL);
                duration += std::chrono::duration_cast<microseconds>(Clock::now() - init_calc);
                num_calcs++;
            }

            FL.th1=jointLimiter(FL.th1, -110, -20);
            FL.th2=jointLimiter(FL.th2, -5, 130);
            FL.th3=jointLimiter(FL.th3, -180, 0);

            IK_order.data.clear();

            IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, MR.th1, 52, MR.th2, 53, MR.th3,
                                    61, BR.th1, 62, BR.th2, 63, BR.th3};

            controller_pub.publish(IK_order);
        }
        else if(state==4)
        {
            target.z += 0.001;
            posFL = target.camLeft2morf(); 
            posFL = posFL.morf2FL(); 
            if(type==0)
            {
                init_calc = Clock::now();
                FL.calcIK(posFL);
                duration += std::chrono::duration_cast<microseconds>(Clock::now() - init_calc);
                num_calcs++;
            }
            else if(type==1)
            {
                init_calc = Clock::now();
                FL.calcNN(posFL);
                duration += std::chrono::duration_cast<microseconds>(Clock::now() - init_calc);
                num_calcs++;
                auxFL.calcIK(posFL);
            }

            FL.th1=jointLimiter(FL.th1, -110, -20);
            FL.th2=jointLimiter(FL.th2, -5, 130);
            FL.th3=jointLimiter(FL.th3, -180, 0);
            //ROS_INFO("%f, %f, %f", FL.th1, FL.th2, FL.th3);

            IK_order.data.clear();

            IK_order.data =    {11, FL.th1, 12, FL.th2, 13, FL.th3,
                                    21, ML.th1, 22, ML.th2, 23, ML.th3,
                                    31, BL.th1, 32, BL.th2, 33, BL.th3,
                                    41, FR.th1, 42, FR.th2, 43, FR.th3,
                                    51, MR.th1, 52, MR.th2, 53, MR.th3,
                                    61, BR.th1, 62, BR.th2, 63, BR.th3};

            controller_pub.publish(IK_order);
        }

        std::cout << state << " , " << stereo.target.z << std::endl;


        
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
