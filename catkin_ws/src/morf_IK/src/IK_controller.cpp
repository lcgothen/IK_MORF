// PARA CORRER
// . ~/tese/catkin_ws/devel/setup.bash
// cd home/leonor/tese/IK_MORF/catkin_ws/devel/lib/morf_IK
// ./IK_controller

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cmath>

class PointCoord
{
    public:
    float x, y, z;
    float Ox, Oy, Oz;
 
    PointCoord world2TC0() // change coordinates from world frame to TC0 frame
    {
        PointCoord aux;

        // find a way to get the values automatically
        aux.x = -y+0.26857;
        aux.y = x-0.10079;
        aux.z = z-0.043091;

        return aux;
    }

    PointCoord morf2FL() // change coordinates from morf frame to front left leg frame
    {
        PointCoord aux, aux0, aux1;

        /*aux.x = -z-0.15904;
        aux.y = y-0.097885;
        aux.z = x+0.009518;*/

        aux0.x=z+1.5904e-01;
        aux0.y=x+1.0487e-02;
        aux0.z=y-6.2794e-02;

        aux1.x=-aux0.x+7.3954e-05;
        aux1.y=-aux0.y-2.1711e-03;
        aux1.z=aux0.z-2.3107e-02;

        aux.x=aux1.x+6.9797e-05;
        aux.y=aux1.z-1.1983e-02;
        aux.z=-aux1.y-1.2019e-03;

        return aux;
    }

    PointCoord morf2ML() // change coordinates from morf frame to middle left leg frame
    {
        PointCoord aux, aux0, aux1;

        aux0.x=z+1.5041e-02;
        aux0.y=x+1.0487e-02;
        aux0.z=y-6.2780e-02;

        aux1.x=-aux0.x+7.3954e-05;
        aux1.y=-aux0.y-2.1711e-03;
        aux1.z=aux0.z-2.3107e-02;

        aux.x=aux1.x+6.9797e-05;
        aux.y=aux1.z-1.1983e-02;
        aux.z=-aux1.y-1.2019e-03;

        return aux;
    }

    PointCoord morf2BL() // change coordinates from morf frame to back left leg frame
    {
        PointCoord aux, aux0, aux1;

        aux0.x=z-1.2896e-01;
        aux0.y=x+1.0487e-02;
        aux0.z=y-6.2775e-02;

        aux1.x=-aux0.x+7.3954e-05;
        aux1.y=-aux0.y-2.1711e-03;
        aux1.z=aux0.z-2.3107e-02;

        aux.x=aux1.x+6.9797e-05;
        aux.y=aux1.z-1.1983e-02;
        aux.z=-aux1.y-1.2019e-03;

        return aux;
    }

    PointCoord morf2FR() // change coordinates from morf frame to front right leg frame
    {
        PointCoord aux, aux0, aux1;

        aux0.x=-z-1.5942e-01;
        aux0.y=x+1.0488e-02;
        aux0.z=-y-6.2777e-02;

        aux1.x=-aux0.x+7.3954e-05;
        aux1.y=-aux0.y-2.1711e-03;
        aux1.z=aux0.z-2.3107e-02;

        aux.x=aux1.x+6.9648e-05;
        aux.y=aux1.z-1.1983e-02;
        aux.z=-aux1.y-1.2020e-03;

        return aux;
    }

    PointCoord morf2MR() // change coordinates from morf frame to middle right leg frame
    {
        PointCoord aux, aux0, aux1;

        aux0.x=-z-1.4819e-02;
        aux0.y=x+1.0488e-02;
        aux0.z=-y-6.2778e-02;

        aux1.x=-aux0.x+7.3954e-05;
        aux1.y=-aux0.y-2.1711e-03;
        aux1.z=aux0.z-2.3107e-02;

        aux.x=aux1.x+6.9648e-05;
        aux.y=aux1.z-1.1983e-02;
        aux.z=-aux1.y-1.2020e-03;

        return aux;
    }

    PointCoord morf2BR() // change coordinates from morf frame to back right leg frame
    {
        PointCoord aux, aux0, aux1;

        aux0.x=-z+1.2918e-01;
        aux0.y=x+1.0488e-02;
        aux0.z=-y-6.2779e-02;

        aux1.x=-aux0.x+7.3954e-05;
        aux1.y=-aux0.y-2.1711e-03;
        aux1.z=aux0.z-2.3107e-02;

        aux.x=aux1.x+6.9648e-05;
        aux.y=aux1.z-1.1983e-02;
        aux.z=-aux1.y-1.2020e-03;

        return aux;
    }
};

class IK_angles
{
    public:
    float th1, th2, th3;
 
    void get_angles(PointCoord target) // calculate angles with IK equations, from target in TC0 frame
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

        /* OLD IK calcs
        xc1 = target.y-L1*cos(th1);
        yc1 = target.z-L0;
        zc1 = target.x;

        th2 = acos((pow(L3,2)-pow(xc1,2)-pow(yc1,2)-pow(L2,2))/(-2*L2*sqrt(pow(xc1,2)+pow(yc1,2))))-atan2(-yc1,xc1)+offset2;
        th3 = acos((pow(xc1,2)+pow(yc1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3))+offset3;
        */
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IK_controller");

    ros::NodeHandle n;

    ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("joints_target", 1000);

    // target coords
    PointCoord target;
    
    target.x = -5.9516e-02;
    target.y = +1.9789e-01;
    target.z = -2.0904e-01;
    
    
    PointCoord targetFL = target.morf2FL(); //convert to front left leg frame

    ROS_INFO("%f, %f, %f", targetFL.x, targetFL.y, targetFL.z);


    IK_angles FL;
    FL.get_angles(targetFL);
    ROS_INFO("%f, %f, %f", FL.th1, FL.th2, FL.th3);

    /* COMM TEST */

    ros::Rate loop_rate(10);
    
    std_msgs::Float32MultiArray msg;
    float angle = 0;
    while (ros::ok())
    {
        msg.data.clear();
        msg.data.push_back(FL.th1);
        msg.data.push_back(FL.th2);
        msg.data.push_back(FL.th3);

        //ROS_INFO("%f, %f, %f", msg.data[0], msg.data[1], msg.data[2]);

        controller_pub.publish(msg);
        angle = angle+0.1;

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
