#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"

float increment = -0.1;
float speed = 1;
void timerCallback(const ros::TimerEvent& event){
    increment *= -1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "walking_test");
    ros::NodeHandle n;
    ros::Publisher femur_rf_pub = n.advertise<std_msgs::Float64>("/ardent/j_femur_rf_position_controller/command",1000);
    ros::Publisher femur_rm_pub = n.advertise<std_msgs::Float64>("/ardent/j_femur_rm_position_controller/command",1000);
    ros::Publisher femur_rr_pub = n.advertise<std_msgs::Float64>("/ardent/j_femur_rr_position_controller/command",1000);
    ros::Publisher femur_lf_pub = n.advertise<std_msgs::Float64>("/ardent/j_femur_lf_position_controller/command",1000);
    ros::Publisher femur_lm_pub = n.advertise<std_msgs::Float64>("/ardent/j_femur_lm_position_controller/command",1000);
    ros::Publisher femur_lr_pub = n.advertise<std_msgs::Float64>("/ardent/j_femur_lr_position_controller/command",1000);
    
    ros::Publisher tibia_rf_pub = n.advertise<std_msgs::Float64>("/ardent/j_tibia_rf_position_controller/command",1000);
    ros::Publisher tibia_rm_pub = n.advertise<std_msgs::Float64>("/ardent/j_tibia_rm_position_controller/command",1000);
    ros::Publisher tibia_rr_pub = n.advertise<std_msgs::Float64>("/ardent/j_tibia_rr_position_controller/command",1000);
    ros::Publisher tibia_lf_pub = n.advertise<std_msgs::Float64>("/ardent/j_tibia_lf_position_controller/command",1000);
    ros::Publisher tibia_lm_pub = n.advertise<std_msgs::Float64>("/ardent/j_tibia_lm_position_controller/command",1000);
    ros::Publisher tibia_lr_pub = n.advertise<std_msgs::Float64>("/ardent/j_tibia_lr_position_controller/command",1000);
    
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
    float angle = 0;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        std_msgs::Float64 femur_msg;
        
        femur_msg.data = angle;

        std_msgs::Float64 tibia_msg;
        tibia_msg.data = -femur_msg.data;
        
        femur_rf_pub.publish(femur_msg);
        femur_rm_pub.publish(femur_msg);
        femur_rr_pub.publish(femur_msg);
        femur_lf_pub.publish(femur_msg);
        femur_lm_pub.publish(femur_msg);
        femur_lr_pub.publish(femur_msg);

        tibia_rf_pub.publish(tibia_msg);
        tibia_rm_pub.publish(tibia_msg);
        tibia_rr_pub.publish(tibia_msg);
        tibia_lf_pub.publish(tibia_msg);
        tibia_lm_pub.publish(tibia_msg);
        tibia_lr_pub.publish(tibia_msg);
        ros::spinOnce();
        loop_rate.sleep();
        angle += increment;
    }

    return 0;
}