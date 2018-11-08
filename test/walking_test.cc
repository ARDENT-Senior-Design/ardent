#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "../include/ardent.h"

float increment = -0.1;
float speed = 1;
void timerCallback(const ros::TimerEvent& event){
    increment *= -1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "walking_test");
    ros::NodeHandle n;
    ardent::ArdentRobot ardent; 
    
    Eigen::Vector3d command = Eigen::Vector3d(0.5,0,0.5);

    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
    float angle = 0;
    ros::Rate loop_rate(10);

    while(ros::ok())
    {

        ardent.PublishLegPosition("rm",command);
        ardent.PublishLegPosition("rf",command);
        ardent.PublishLegPosition("rr",command);
        ardent.PublishLegPosition("lm",command);
        ardent.PublishLegPosition("lr",command);
        ardent.PublishLegPosition("lf",command);

        ros::spinOnce();
        loop_rate.sleep();
        // angle += increment;
    }

    return 0;
}
