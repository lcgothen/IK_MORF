#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>

#include "coordinates.hpp"
using namespace coords;

#include "controller.hpp"
using namespace controller;

void angles::get_angles(point target) // calculate angles with IK equations
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