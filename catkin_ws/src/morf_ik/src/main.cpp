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

extern "C" {
    #include "extApi.h"
}


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;

/********************* USAGE *************************

./main TYPE N_TRIALS

TYPE_: 0 means equations, 1 means neural network
N_TRIALS: number of trials

******************************************************/


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cout << "./main TYPE N_TRIALS" << std::endl;
        exit(EXIT_FAILURE);
    }

    int type = std::stoi(argv[1]);
    int n_trials = std::stoi(argv[2]);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream timeStream;
    timeStream << "./results/" << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S/");
    std::string results_path = timeStream.str();
    
    mkdir(results_path.c_str(),0777);

    std::ofstream resFile, failFile;
    std::string resName = results_path+std::string("successes.data");
    std::string failName = results_path+std::string("failures.data");

    std::cout << results_path << std::endl;

    resFile.open(resName);
    failFile.open(failName);

    simxInt clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);

    for(int trial=0; trial < n_trials; trial++)
    {
        robot morf;
        images stereo;

        std::string ann_path = "./neural_networks/data_4div_direct/batch_01_10_01_50000_02_09/";

        ros::init(argc, argv, "IK_controller");
        ros::NodeHandle n;

        ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);

        ros::Subscriber jointPos_sub = n.subscribe("joint_positions", 1000, &robot::jointPosCallback, &morf);
        ros::Subscriber forceSens_sub = n.subscribe("force_sensors", 1000, &robot::forceSensCallback, &morf);
        ros::Subscriber foot2button_sub = n.subscribe("foot2button_position", 1000, &robot::foot2buttonPosCallback, &morf);

        image_transport::ImageTransport it(n);
        image_transport::Subscriber imageLeft_sub = it.subscribe("imageLeft", 1, &images::imageLeftCallback, &stereo);
        image_transport::Subscriber imageRight_sub = it.subscribe("imageRight", 1, &images::imageRightCallback, &stereo);
        image_transport::Subscriber generalImg_sub = it.subscribe("imageGeneral", 1, &images::generalImgCallback, &stereo);


        
        float distance;
        float but_rad = 0.02;
        bool distFail = false, durFail = false;

        std::chrono::microseconds duration=std::chrono::microseconds(0);
        std::chrono::microseconds trial_dur;
        float num_calcs=0;
        typedef std::chrono::high_resolution_clock Clock;
        typedef std::chrono::microseconds microseconds;
        Clock::time_point init = Clock::now();
        Clock::time_point init_calc;

        simxStartSimulation(clientID, simx_opmode_oneshot);

        // target coords for stabilizing with 4 legs
        point posFL, posFR, stableML, stableMR, stableBL, stableBR;
        angles FL, FR, ML, MR, BL, BR, auxFL;
        angles auxML, auxMR, auxBL, auxBR;
        
        posFL.x = 0;//-7.5776e-02;
        posFL.y = +1.3090e-01;
        posFL.z = -2.9912e-01;

        stableML.x = -7.5776e-02;
        stableML.y = +1.7632e-01;
        stableML.z = -1.6912e-01;

        stableBL.x = -7.5776e-02;
        stableBL.y = 0.17579;
        stableBL.z = 0.085760;

        posFR.x = 0;//-7.5776e-02;
        posFR.y = -1.3090e-01;
        posFR.z = -2.9912e-01;

        stableMR.x = -7.5776e-02;
        stableMR.y = -1.7632e-01;
        stableMR.z = -1.6912e-01;

        stableBR.x = -7.5776e-02;
        stableBR.y = -0.17579;
        stableBR.z = 0.085760;

        // calculate FL angles according to specified algorithm
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
            FL.calcNN(posFL, &coords::point::morf2FL, ann_path);
            duration += std::chrono::duration_cast<microseconds>(Clock::now() - init_calc);
            num_calcs++;
        }

        // convert to leg frames for the rest of the legs
        stableML = stableML.morf2ML(); 
        stableBL = stableBL.morf2BL(); 
        posFR = posFR.morf2FR(); 
        stableMR = stableMR.morf2MR(); 
        stableBR = stableBR.morf2BR(); 

        // calculate IK parameters for the rest of the legs
        ML.calcIK(stableML);
        BL.calcIK(stableBL);
        FR.calcIK(posFR);
        MR.calcIK(stableMR);
        BR.calcIK(stableBR);

        auxML.th1=-1.100229;
        auxML.th2=2.324319;
        auxML.th3=-0.2;
        auxMR.th1=1.100229;
        auxMR.th2=2.324319;
        auxMR.th3=-0.2;

        auxBL.th1=-0.507609;
        auxBL.th2=2.224319;
        auxBL.th3=0;
        auxBR.th1=0.507609;
        auxBR.th2=2.224319;
        auxBR.th3=0;

        angles default_;
        default_.th1=0;
        default_.th2=2.024319;
        default_.th3=0;


        ros::Rate loop_rate(10);
        
        std_msgs::Float32MultiArray IK_order;
        CPG cpg;
        bool stable=false, inPos=false, done=false;
        int state=0, stable_state=0;
        point target;

        //int nearZ=0;

        // -110 < th1 < -20
        // jointValues.at(CF0)=jointLimiter(jointValues.at(CF0), -5, 130);
        // jointValues.at(FT0)=jointLimiter(jointValues.at(FT0), -80, 0);
    
        while(ros::ok() && simxGetConnectionId(clientID)!=-1)
        {
            if(!stereo.imageL.empty() && !stereo.imageR.empty() && !stable)
            {
                stereo.match();
                //stereo.blob();
            }

            //ROS_INFO("target: %f, %f, %f\n actual: %f, %f, %f", ML.th1, ML.th2, ML.th3, morf.ML.th1, morf.ML.th2, morf.ML.th3);
            IK_order.data.clear();

            // if(stereo.target.z >= 0.15)
            //     nearZ=0;
            // else
            //     nearZ++;

            // std::cout << state << std::endl;

            trial_dur = std::chrono::duration_cast<microseconds>(Clock::now() - init); //difftime(time(NULL), init);

            if(state==0 && !stereo.imageL.empty() && !stereo.imageR.empty())
                state=1;
            else if(state==1 && stereo.nearZ)
                state=2;
            else if(state==2 && stable)
                state=3;
            else if(state==3 && morf.FL.th1<FL.th1+0.001 && morf.FL.th1>FL.th1-0.001 && 
                    morf.FL.th2<FL.th2+0.001 && morf.FL.th2>FL.th2-0.001 &&
                    morf.FL.th3<FL.th3+0.001 && morf.FL.th3>FL.th3-0.001)
                state=4;
            else if(state==4 && morf.sensFL>0.1)
                state=5;
            else if(trial_dur.count()>120000000)//(clock()-init>120*CLOCKS_PER_SEC)
                state=6;

            if(state==0)
            {
                // left leg angles
                IK_order.data.push_back(default_.th1);
                IK_order.data.push_back(default_.th2);
                IK_order.data.push_back(default_.th3);
                IK_order.data.push_back(default_.th1);
                IK_order.data.push_back(default_.th2);
                IK_order.data.push_back(default_.th3);
                IK_order.data.push_back(default_.th1);
                IK_order.data.push_back(default_.th2);
                IK_order.data.push_back(default_.th3);

                // right leg angles
                IK_order.data.push_back(default_.th1);
                IK_order.data.push_back(default_.th2);
                IK_order.data.push_back(default_.th3);
                IK_order.data.push_back(default_.th1);
                IK_order.data.push_back(default_.th2);
                IK_order.data.push_back(default_.th3);
                IK_order.data.push_back(default_.th1);
                IK_order.data.push_back(default_.th2);
                IK_order.data.push_back(default_.th3);

                controller_pub.publish(IK_order);
            }
            else if(state==1)
            {
                cpg.cyclic(&stereo);
                cpg.walk(stereo);

                // left leg angles
                IK_order.data.push_back(cpg.FL.th1);
                IK_order.data.push_back(cpg.FL.th2);
                IK_order.data.push_back(cpg.FL.th3);
                IK_order.data.push_back(cpg.ML.th1);
                IK_order.data.push_back(cpg.ML.th2);
                IK_order.data.push_back(cpg.ML.th3);
                IK_order.data.push_back(cpg.BL.th1);
                IK_order.data.push_back(cpg.BL.th2);
                IK_order.data.push_back(cpg.BL.th3);

                // right leg angles
                IK_order.data.push_back(cpg.FR.th1);
                IK_order.data.push_back(cpg.FR.th2);
                IK_order.data.push_back(cpg.FR.th3);
                IK_order.data.push_back(cpg.MR.th1);
                IK_order.data.push_back(cpg.MR.th2);
                IK_order.data.push_back(cpg.MR.th3);
                IK_order.data.push_back(cpg.BR.th1);
                IK_order.data.push_back(cpg.BR.th2);
                IK_order.data.push_back(cpg.BR.th3);

                controller_pub.publish(IK_order);
            }
            else if(state==2)
            {
                if(stable_state==0 && morf.FL.th1<FL.th1+0.001 && morf.FL.th1>FL.th1-0.001 && 
                        morf.FL.th2<FL.th2+0.001 && morf.FL.th2>FL.th2-0.001 &&
                        morf.FL.th3<FL.th3+0.001 && morf.FL.th3>FL.th3-0.001)
                    stable_state=1;
                else if(stable_state==1 && morf.BL.th1<auxBL.th1+0.001 && morf.BL.th1>auxBL.th1-0.001 && 
                        morf.BL.th2<auxBL.th2+0.001 && morf.BL.th2>auxBL.th2-0.001 &&
                        morf.BL.th3<auxBL.th3+0.001 && morf.BL.th3>auxBL.th3-0.001)
                    stable_state=2;
                else if(stable_state==2 && morf.BL.th1<BL.th1+0.001 && morf.BL.th1>BL.th1-0.001 && 
                        morf.BL.th2<BL.th2+0.001 && morf.BL.th2>BL.th2-0.001 &&
                        morf.BL.th3<BL.th3+0.001 && morf.BL.th3>BL.th3-0.001)
                    stable_state=3;
                else if(stable_state==3 && morf.ML.th1<auxML.th1+0.001 && morf.ML.th1>auxML.th1-0.001 && 
                        morf.ML.th2<auxML.th2+0.001 && morf.ML.th2>auxML.th2-0.001 &&
                        morf.ML.th3<auxML.th3+0.001 && morf.ML.th3>auxML.th3-0.001)
                    stable_state=4;
                else if(stable_state==4 && morf.ML.th1<ML.th1+0.001 && morf.ML.th1>ML.th1-0.001 && 
                        morf.ML.th2<ML.th2+0.001 && morf.ML.th2>ML.th2-0.001 &&
                        morf.ML.th3<ML.th3+0.001 && morf.ML.th3>ML.th3-0.001)
                    stable_state=5;
                else if(stable_state==5 && morf.FR.th1<FR.th1+0.001 && morf.FR.th1>FR.th1-0.001 && 
                        morf.FR.th2<FR.th2+0.001 && morf.FR.th2>FR.th2-0.001 &&
                        morf.FR.th3<FR.th3+0.001 && morf.FR.th3>FR.th3-0.001)
                    stable_state=6;
                else if(stable_state==6 && morf.BR.th1<auxBR.th1+0.001 && morf.BR.th1>auxBR.th1-0.001 && 
                        morf.BR.th2<auxBR.th2+0.001 && morf.BR.th2>auxBR.th2-0.001 &&
                        morf.BR.th3<auxBR.th3+0.001 && morf.BR.th3>auxBR.th3-0.001)
                    stable_state=7;
                else if(stable_state==7 && morf.BR.th1<BR.th1+0.001 && morf.BR.th1>BR.th1-0.001 && 
                        morf.BR.th2<BR.th2+0.001 && morf.BR.th2>BR.th2-0.001 &&
                        morf.BR.th3<BR.th3+0.001 && morf.BR.th3>BR.th3-0.001)
                    stable_state=8;
                else if(stable_state==8 && morf.MR.th1<auxMR.th1+0.001 && morf.MR.th1>auxMR.th1-0.001 && 
                        morf.MR.th2<auxMR.th2+0.001 && morf.MR.th2>auxMR.th2-0.001 &&
                        morf.MR.th3<auxMR.th3+0.001 && morf.MR.th3>auxMR.th3-0.001)
                    stable_state=9;
                else if(stable_state==9 && morf.MR.th1<MR.th1+0.001 && morf.MR.th1>MR.th1-0.001 && 
                        morf.MR.th2<MR.th2+0.001 && morf.MR.th2>MR.th2-0.001 &&
                        morf.MR.th3<MR.th3+0.001 && morf.MR.th3>MR.th3-0.001)
                    stable_state=10;

                if(stable_state==0)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);

                    // right leg angles
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                }
                else if(stable_state==1)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(auxBL.th1);
                    IK_order.data.push_back(auxBL.th2);
                    IK_order.data.push_back(auxBL.th3);


                    // right leg angles
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                }
                else if(stable_state==2)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);


                    // right leg angles
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                }
                else if(stable_state==3)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(auxML.th1);
                    IK_order.data.push_back(auxML.th2);
                    IK_order.data.push_back(auxML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);

                    // right leg angles
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                }
                else if(stable_state==4)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(ML.th1);
                    IK_order.data.push_back(ML.th2);
                    IK_order.data.push_back(ML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);

                    // right leg angles
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                }
                else if(stable_state==5)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(ML.th1);
                    IK_order.data.push_back(ML.th2);
                    IK_order.data.push_back(ML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);


                    // right leg angles
                    IK_order.data.push_back(FR.th1);
                    IK_order.data.push_back(FR.th2);
                    IK_order.data.push_back(FR.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                }
                else if(stable_state==6)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(ML.th1);
                    IK_order.data.push_back(ML.th2);
                    IK_order.data.push_back(ML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);


                    // right leg angles
                    IK_order.data.push_back(FR.th1);
                    IK_order.data.push_back(FR.th2);
                    IK_order.data.push_back(FR.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(auxBR.th1);
                    IK_order.data.push_back(auxBR.th2);
                    IK_order.data.push_back(auxBR.th3);
                }
                else if(stable_state==7)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(ML.th1);
                    IK_order.data.push_back(ML.th2);
                    IK_order.data.push_back(ML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);


                    // right leg angles
                    IK_order.data.push_back(FR.th1);
                    IK_order.data.push_back(FR.th2);
                    IK_order.data.push_back(FR.th3);
                    IK_order.data.push_back(0);
                    IK_order.data.push_back(default_.th2);
                    IK_order.data.push_back(default_.th3);
                    IK_order.data.push_back(BR.th1);
                    IK_order.data.push_back(BR.th2);
                    IK_order.data.push_back(BR.th3);
                }
                else if(stable_state==8)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(ML.th1);
                    IK_order.data.push_back(ML.th2);
                    IK_order.data.push_back(ML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);


                    // right leg angles
                    IK_order.data.push_back(FR.th1);
                    IK_order.data.push_back(FR.th2);
                    IK_order.data.push_back(FR.th3);
                    IK_order.data.push_back(auxMR.th1);
                    IK_order.data.push_back(auxMR.th2);
                    IK_order.data.push_back(auxMR.th3);
                    IK_order.data.push_back(BR.th1);
                    IK_order.data.push_back(BR.th2);
                    IK_order.data.push_back(BR.th3);
                }
                else if(stable_state==9)
                {
                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(ML.th1);
                    IK_order.data.push_back(ML.th2);
                    IK_order.data.push_back(ML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);


                    // right leg angles
                    IK_order.data.push_back(FR.th1);
                    IK_order.data.push_back(FR.th2);
                    IK_order.data.push_back(FR.th3);
                    IK_order.data.push_back(MR.th1);
                    IK_order.data.push_back(MR.th2);
                    IK_order.data.push_back(MR.th3);
                    IK_order.data.push_back(BR.th1);
                    IK_order.data.push_back(BR.th2);
                    IK_order.data.push_back(BR.th3);
                }
                else if(stable_state==10)
                {
                    stable=true;

                    // left leg angles
                    IK_order.data.push_back(FL.th1);
                    IK_order.data.push_back(FL.th2);
                    IK_order.data.push_back(FL.th3);
                    IK_order.data.push_back(ML.th1);
                    IK_order.data.push_back(ML.th2);
                    IK_order.data.push_back(ML.th3);
                    IK_order.data.push_back(BL.th1);
                    IK_order.data.push_back(BL.th2);
                    IK_order.data.push_back(BL.th3);


                    // right leg angles
                    IK_order.data.push_back(FR.th1);
                    IK_order.data.push_back(FR.th2);
                    IK_order.data.push_back(FR.th3);
                    IK_order.data.push_back(MR.th1);
                    IK_order.data.push_back(MR.th2);
                    IK_order.data.push_back(MR.th3);
                    IK_order.data.push_back(BR.th1);
                    IK_order.data.push_back(BR.th2);
                    IK_order.data.push_back(BR.th3);
                }

                controller_pub.publish(IK_order);
            }
            else if(state==3)
            {
                target.x=stereo.target_avg.x;
                target.y=stereo.target_avg.y;
                target.z=stereo.target_avg.z-0.05;

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
                    FL.calcNN(posFL, &coords::point::morf2FL, ann_path);
                    duration += std::chrono::duration_cast<microseconds>(Clock::now() - init_calc);
                    num_calcs++;
                }

                FL.th1=jointLimiter(FL.th1, -110, -20);
                FL.th2=jointLimiter(FL.th2, -5, 130);
                FL.th3=jointLimiter(FL.th3, -180, 0);

                // left leg angles
                IK_order.data.push_back(FL.th1);
                IK_order.data.push_back(FL.th2);
                IK_order.data.push_back(FL.th3);
                IK_order.data.push_back(ML.th1);
                IK_order.data.push_back(ML.th2);
                IK_order.data.push_back(ML.th3);
                IK_order.data.push_back(BL.th1);
                IK_order.data.push_back(BL.th2);
                IK_order.data.push_back(BL.th3);


                // right leg angles
                IK_order.data.push_back(FR.th1);
                IK_order.data.push_back(FR.th2);
                IK_order.data.push_back(FR.th3);
                IK_order.data.push_back(MR.th1);
                IK_order.data.push_back(MR.th2);
                IK_order.data.push_back(MR.th3);
                IK_order.data.push_back(BR.th1);
                IK_order.data.push_back(BR.th2);
                IK_order.data.push_back(BR.th3);

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
                    FL.calcNN(posFL, &coords::point::morf2FL, ann_path);
                    duration += std::chrono::duration_cast<microseconds>(Clock::now() - init_calc);
                    num_calcs++;
                    auxFL.calcIK(posFL);
                }

                FL.th1=jointLimiter(FL.th1, -110, -20);
                FL.th2=jointLimiter(FL.th2, -5, 130);
                FL.th3=jointLimiter(FL.th3, -180, 0);
                //ROS_INFO("%f, %f, %f", FL.th1, FL.th2, FL.th3);

                // left leg angles
                IK_order.data.push_back(FL.th1);
                IK_order.data.push_back(FL.th2);
                IK_order.data.push_back(FL.th3);
                IK_order.data.push_back(ML.th1);
                IK_order.data.push_back(ML.th2);
                IK_order.data.push_back(ML.th3);
                IK_order.data.push_back(BL.th1);
                IK_order.data.push_back(BL.th2);
                IK_order.data.push_back(BL.th3);


                // right leg angles
                IK_order.data.push_back(FR.th1);
                IK_order.data.push_back(FR.th2);
                IK_order.data.push_back(FR.th3);
                IK_order.data.push_back(MR.th1);
                IK_order.data.push_back(MR.th2);
                IK_order.data.push_back(MR.th3);
                IK_order.data.push_back(BR.th1);
                IK_order.data.push_back(BR.th2);
                IK_order.data.push_back(BR.th3);

                //std::cout << stereo.target.x << " , " << stereo.target.y <<  " , " << stereo.target.z << std::endl;
                controller_pub.publish(IK_order);
            }
            else if(state==5)
            {
                distance = sqrt(pow(morf.foot2button.x,2)+pow(morf.foot2button.y,2));

                if(distance > but_rad)
                    distFail=true;

                break;
            }
            else if(state==6)
            {
                durFail=true;
                break;
            }

            ros::spinOnce();

            loop_rate.sleep();
        }

        std::string imgName = results_path+"trial"+std::to_string(trial)+std::string(".png");
        imwrite(imgName, stereo.genImg);
        
        simxStopSimulation(clientID, simx_opmode_oneshot);
        sleep(1);

        if(!durFail && !distFail)
        {
            std::cout<< "trial " << trial << ": " << "Success! " << distance << std::endl;
            resFile << "trial " << trial << ":\t" << duration.count()/num_calcs << "\t" << distance;

            if(type==1)
            {
                resFile << "\t" << abs(auxFL.th1-FL.th1)/abs(FL.th1)*100 << "\t" << abs(auxFL.th2-FL.th2)/abs(FL.th2)*100 << "\t" << abs(auxFL.th3-FL.th3)/abs(FL.th3)*100;
            }
            
            resFile << "\n";
        }
        else if(durFail)
        {
            std::cout << "Failed duration!" << std::endl;
            failFile << "trial " << trial << ":\t" << "duration " << "\t" << trial_dur.count() << "\n";
        }
        else
        {
            failFile << "trial " << trial << ":\t" << "distance" << "\t" << distance << "\n";
            std::cout << "Failed distance: " << distance << std::endl;
        }
    }

    simxFinish(clientID);
    
    resFile.close();
    failFile.close();


    return 0;
}
