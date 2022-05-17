#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <random>
#include <fstream>
#include <vector>
#include <string>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;

std::vector<float> calcIK_unsafe(point target) // calculate angles with IK equations
{
    float xc1, yc1, zc1;
    float L0 = 0.017011, L1 = 0.039805, L2 = 0.070075, L3=0.11542;// L3 = 0.11243;
    float offset2 = 0.8, offset3 = -2.5;

    float aux_th1, aux_th2, aux_th3;
    std::vector<float> th;

    aux_th1 = atan2(-target.x, target.y);

    xc1 = cos(aux_th1)*target.x+sin(aux_th1)*target.y;
    yc1 = -sin(aux_th1)*target.x+cos(aux_th1)*target.y-L1;
    zc1 = target.z+L0;

    // th2 = acos((pow(L3,2)-pow(yc1,2)-pow(zc1,2)-pow(L2,2))/(-2*L2*sqrt(pow(yc1,2)+pow(zc1,2))))+atan2(zc1,yc1)+offset2;
    // th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3))+offset3; 

    aux_th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3));
    aux_th2 = atan2(zc1,yc1)+atan2(L3*sin(aux_th3),L2+L3*cos(aux_th3))+offset2;
    aux_th3 += offset3;
    
    th.push_back(aux_th1);
    th.push_back(aux_th2);
    th.push_back(aux_th3);

    return th;

}


int main(int argc, char **argv)
{
    std::random_device rand_dev;
    unsigned int seed = rand_dev();

    // -7.4826e-02 <= x <= 0.11527
    // +6.2794e-02 <= y <= +3.2290e-01
    // -3.8406e-01 <= z <= -1.6912e-01

    // x_interval = 0.19010
    // y_interval = 0.26011
    // z_interval = 0.21494

    float x_length = -0.25494;//0.23010;
    float y_length = 0.30011;//0.30011;
    float z_length = 0.2301;//0.25494;

    float x_start = 0.225164; //0.205164;//-5.4826e-02;
    float y_start = -0.057090; //-0.01509;//+8.2794e-02;
    float z_start = -0.0433698; //-0.0433698;//-3.6406e-01;


    // float x_length = 0.19010;
    // float y_length = 0.26011;
    // float z_length = 0.21494;

    // float x_start = -7.4826e-02;
    // float y_start = +6.2794e-02;
    // float z_start = -3.8406e-01;

    // point aux, start, finish;
    // aux.x = x_start;
    // aux.y = y_start;
    // aux.z = z_start;

    // start = aux.morf2FL();

    // aux.x = x_start+x_length;
    // aux.y = y_start+y_length;
    // aux.z = z_start+z_length;

    // finish = aux.morf2FL();

    // std::cout << start.x << " , " << start.y << " , " << start.z << std::endl;
    // std::cout << finish.x-start.x << " , " << finish.y-start.y << " , " << finish.z-start.z<< std::endl;

    int div=6;

    float x_step = x_length/div;
    float y_step = y_length/div;
    float z_step = z_length/div;

    std::default_random_engine rand_gen(seed);
    // std::uniform_real_distribution<float> x_interval(-7.4826e-02,0.11527);
    // std::uniform_real_distribution<float> y_interval(+6.2794e-02,+3.2290e-01);
    // std::uniform_real_distribution<float> z_interval(-3.8406e-01,-1.6912e-01);
    // std::uniform_real_distribution<float> x_interval(-7.4826e-02, -0.036806);
    // std::uniform_real_distribution<float> y_interval(+6.2794e-02, 0.11482);
    // std::uniform_real_distribution<float> z_interval(-3.8406e-01, -0.34107);

    // create train data
    for(int j=0; j<div; j++)
    {
        std::uniform_real_distribution<float> x_interval(x_start, x_start+x_step);
        y_start = -0.057090; //-0.01509; //+6.2794e-02;
        for(int k=0; k<div; k++)
        {
            std::uniform_real_distribution<float> y_interval(y_start, y_start+y_step);
            z_start = -0.0433698; //-3.8406e-01;
            for(int l=0; l<div; l++)
            {
                std::uniform_real_distribution<float> z_interval(z_start, z_start+z_step);

                int count=0;
                std::vector<point> input;
                std::vector<angles> output;

                for(int i=0; i<10000; i++)
                {
                    point in;
                    in.x = x_interval(rand_gen);
                    in.y = y_interval(rand_gen);
                    in.z = z_interval(rand_gen);

                    //point in;
                    angles out;
                    std::vector<float> th;
                    //in = aux_in.morf2FL();
                    th = calcIK_unsafe(in);
                    out.th1=th[0];
                    out.th2=th[1];
                    out.th3=th[2];

                    if(!isnan(out.th1) && !isnan(out.th2) && !isnan(out.th3))
                    {
                        count++;
                        //std::cout << count << ": " << out.th1 << "," << out.th2 << "," << out.th3 << std::endl;
                        input.push_back(in);

                        output.push_back(out);
                        //std::cout << count << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
                    }
                }
                
                if(count>50)
                {
                    std::ofstream data;
                    std::string filename = "./neural_networks/data_6div_direct/vali"+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".data");
                    std::cout << filename << std::endl;

                    data.open(filename);

                    data << count << " 3 3\n";

                    for(int i=0; i<count; i++)
                    {
                        data << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
                        data << output[i].th1 << " " << output[i].th2<< " " << output[i].th3 << "\n";
                        // std::cout << i << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
                    }

                    data.close();
                }

                z_start+=z_step;
            }

            y_start+=y_step;
        }

        x_start+=x_step;
    }


    return 0;
}
