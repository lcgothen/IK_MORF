#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <random>

#include "coordinates.hpp"
using namespace coords;

#include "controller.hpp"
using namespace controller;

void angles::calcIK(point target) // calculate angles with IK equations
{
    float xc1, yc1, zc1;
    float L0 = 0.01701, L1 = 0.039805, L2 = 0.070044, L3 = 0.11242;
    float offset2 = 1.5, offset3 = -2.25;

    th1 = atan2(-target.x, target.y);

    xc1 = cos(th1)*target.x+sin(th1)*target.y-L1;
    yc1 = sin(th1)*target.x+cos(th1)*target.y;
    zc1 = target.z-L0;

    th2 = acos((pow(L3,2)-pow(yc1,2)-pow(zc1,2)-pow(L2,2))/(-2*L2*sqrt(pow(yc1,2)+pow(zc1,2))))-atan2(-zc1,yc1)+offset2;
    th3 = acos((pow(yc1,2)+pow(zc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3))+offset3;
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

void CPG::walk()
{
    float offset2 = 1.5, offset3 = -2.25/4;
    // back right
    BR.th1 = oH1; 
    BR.th2 = oH2+offset2; 
    BR.th3 = offset3; 

    // back left
    BL.th1 = oH1;
    BL.th2 = -oH2+offset2;
    BL.th3 = offset3;

    // middle right
    MR.th1 = -oH1;
    MR.th2 = -oH2+offset2;
    MR.th3 = offset3;
 
    // middle left
    ML.th1 = -oH1;
    ML.th2 = oH2+offset2;
    ML.th3 = offset3;

    // front right
    FR.th1 = oH1;
    FR.th2 = oH2+offset2;
    FR.th3 = offset3;

    // front left
    FL.th1 = oH1;
    FL.th2 = -oH2+offset2;
    FL.th3 = offset3;
}

void DMP::init(point y0, float Hz)
{
    for(int i=0; i<N; i++)
    {
        w[i] = rand()/RAND_MAX;
        h[i] = rand()/RAND_MAX;
        c[i] = rand()/RAND_MAX;
    }

    dy.x=0;
    dy.y=0;
    dy.z=0;
    ddy.x=0;
    ddy.y=0;
    ddy.z=0;   

    y.x=y0.x;
    y.y=y0.y;
    y.z=y0.z;

    dt = 1/Hz;
}

void DMP::calc(point y0, point goal)
{
    t+= dt;
    dy = y.subtract(prev_y).div_num(dt);
    ddy = dy.subtract(prev_dy).div_num(dt);
    prev_y.x=y.x;
    prev_y.y=y.y;
    prev_y.z=y.z;
    prev_dy.x=dy.x;
    prev_dy.y=dy.y;
    prev_dy.z=dy.z;

    x = exp(-ax*t);

    float sum_mult=0, sum=0;
    for(int i=0; i<N; i++)
    {
        sum_mult += psi[i]*w[i];
        sum += psi[i];
        psi[i] = exp(-h[i]*pow(x-c[i],2));
    }

    point f = goal.subtract(y0).mult_num(sum_mult/sum*x);

    y = goal.subtract((ddy.subtract(ddy).div_num(az)).sum(dy).div_num(bz));
}