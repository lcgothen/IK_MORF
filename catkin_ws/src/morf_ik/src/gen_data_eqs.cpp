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

    float x_length = 0.26494;
    float y_length = 0.25011;
    float z_length = 0.2301;

    float x_start = -0.034776; 
    float y_start = -0.078090;
    float z_start = -0.0433698;

    int div=1;

    float x_step = x_length/div;
    float y_step = y_length/div;
    float z_step = z_length/div;

    std::default_random_engine rand_gen(seed);

    // create train data
    for(int j=0; j<div; j++)
    {
        std::uniform_real_distribution<float> x_interval(x_start, x_start+x_step);
        y_start = -0.057090;
        for(int k=0; k<div; k++)
        {
            std::uniform_real_distribution<float> y_interval(y_start, y_start+y_step);
            z_start = -0.0433698; 
            for(int l=0; l<div; l++)
            {
                std::uniform_real_distribution<float> z_interval(z_start, z_start+z_step);

                int count=0;
                std::vector<point> input;
                std::vector<angles> output;

                for(int i=0; i<100000; i++)
                {
                    point in;
                    in.x = x_interval(rand_gen);
                    in.y = y_interval(rand_gen);
                    in.z = z_interval(rand_gen);

                    //point in;
                    angles out;
                    std::vector<float> th;
                    
                    th = calcIK_unsafe(in);
                    out.th1=th[0];
                    out.th2=th[1];
                    out.th3=th[2];

                    if(!isnan(out.th1) && !isnan(out.th2) && !isnan(out.th3))
                    {
                        count++;
                        
                        input.push_back(in);
                        output.push_back(out);
                    }
                }
                
                if(count>50)
                {
                    std::ofstream data;
                    std::string filename = "./neural_networks/data_"+std::to_string(div)+"div_direct/vali"+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".data");
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
