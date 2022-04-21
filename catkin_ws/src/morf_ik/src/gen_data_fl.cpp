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


int main(int argc, char **argv)
{
    std::vector<point> input;
    std::vector<angles> output;

    std::random_device rand_dev;
    unsigned int seed = rand_dev();

    // -7.4826e-02 <= x <= 0.11527
    // +6.2794e-02 <= y <= +3.2290e-01
    // -3.8406e-01 <= z <= -1.6912e-01

    // x_interval = 0.19010
    // y_interval = 0.26011
    // z_interval = 0.21494

    float x_length = 0.19010;
    float y_length = 0.26011;
    float z_length = 0.21494;

    float x_start = -7.4826e-02;
    float y_start = +6.2794e-02;
    float z_start = -3.8406e-01;

    int div=8;

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
        y_start = +6.2794e-02;
        for(int k=0; k<div; k++)
        {
            std::uniform_real_distribution<float> y_interval(y_start, y_start+y_step);
            z_start = -3.8406e-01;
            for(int l=0; l<div; l++)
            {
                std::uniform_real_distribution<float> z_interval(z_start, z_start+z_step);

                int count=0;

                for(int i=0; i<10000; i++)
                {
                    point aux_in;
                    aux_in.x = x_interval(rand_gen);
                    aux_in.y = y_interval(rand_gen);
                    aux_in.z = z_interval(rand_gen);

                    point in;
                    angles out;
                    in = aux_in.morf2FL();
                    out.calcIK(in);

                    if(!isnan(out.th1) && !isnan(out.th2) && !isnan(out.th3))
                    {
                        count++;
                        //std::cout << count << ": " << out.th1 << "," << out.th2 << "," << out.th3 << std::endl;
                        input.push_back(in);

                        output.push_back(out);
                        //std::cout << count << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
                    }
                }
                
                if(count>0)
                {
                    std::ofstream data;
                    std::string filename = "./data_8div/vali"+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".data");
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

    // int count=0;

    // for(int i=0; i<100000; i++)
    // {
    //     point aux_in;
    //     aux_in.x = x_interval(rand_gen);
    //     aux_in.y = y_interval(rand_gen);
    //     aux_in.z = z_interval(rand_gen);

    //     point in;
    //     angles out;
    //     in = aux_in.morf2FL();
    //     out.calcIK(in);

    //     if(!isnan(out.th1) && !isnan(out.th2) && !isnan(out.th3))
    //     {
    //         count++;
    //         //std::cout << count << ": " << out.th1 << "," << out.th2 << "," << out.th3 << std::endl;
    //         input.push_back(in);

    //         output.push_back(out);
    //         //std::cout << count << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    //     }
    // }

    // std::ofstream data;
    // data.open("data/train.data");

    // data << count << " 3 3\n";

    // for(int i=0; i<count; i++)
    // {
    //     data << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
    //     data << output[i].th1 << " " << output[i].th2<< " " << output[i].th3 << "\n";
    //     std::cout << i << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    // }

    // data.close();


    // create validation data

    // count=0;

    // for(int i=0; i<10000; i++)
    // {
    //     point aux_in;
    //     aux_in.x = x_interval(rand_gen);
    //     aux_in.y = y_interval(rand_gen);
    //     aux_in.z = z_interval(rand_gen);

    //     point in;
    //     angles out;
    //     in = aux_in.morf2FL();
    //     out.calcIK(in);

    //     if(!isnan(out.th1) && !isnan(out.th2) && !isnan(out.th3))
    //     {
    //         count++;
    //         //std::cout << count << ": " << out.th1 << "," << out.th2 << "," << out.th3 << std::endl;
    //         input.push_back(in);

    //         output.push_back(out);
    //         //std::cout << count << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    //     }
    // }

    // data.open("data/vali.data");

    // data << count << " 3 3\n";

    // for(int i=0; i<count; i++)
    // {
    //     data << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
    //     data << output[i].th1 << " " << output[i].th2 << " " << output[i].th3 << "\n";
    //     //std::cout << i << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    // }

    // data.close();

    return 0;
}
