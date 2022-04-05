#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <random>
#include <fstream>
#include <vector>
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

    std::default_random_engine rand_gen(seed);
    // std::uniform_real_distribution<float> x_interval(-7.4826e-02,0.11527);
    // std::uniform_real_distribution<float> y_interval(+6.2794e-02,+3.2290e-01);
    // std::uniform_real_distribution<float> z_interval(-3.8406e-01,-1.6912e-01);
    std::uniform_real_distribution<float> x_interval(-7.4826e-02, -0.027302);
    std::uniform_real_distribution<float> y_interval(+6.2794e-02,0.12782);
    std::uniform_real_distribution<float> z_interval(-3.8406e-01,-0.33032);

    // create train data

    int count=0;

    for(int i=0; i<100000; i++)
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

    std::ofstream data;
    data.open("data/train.data");

    data << count << " 3 3\n";

    for(int i=0; i<count; i++)
    {
        data << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
        data << output[i].th1 << " " << output[i].th2<< " " << output[i].th3 << "\n";
        std::cout << i << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    }

    data.close();


    // create validation data

    count=0;

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

    data.open("data/vali.data");

    data << count << " 3 3\n";

    for(int i=0; i<count; i++)
    {
        data << input[i].x << " " << input[i].y << " " << input[i].z << "\n";
        data << output[i].th1 << " " << output[i].th2 << " " << output[i].th3 << "\n";
        //std::cout << i << ": " << output[i].th1 << "," << output[i].th2 << "," << output[i].th3 << std::endl;
    }

    data.close();

    return 0;
}
