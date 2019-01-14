#include "../include/ardent/leg_kinematics.h"

using namespace ardent_model;

LegKinematics::LegKinematics(std::string new_leg_id, double body_radius)
    : coxa_length(0.1), femur_length(0.2), tibia_length(0.25)
{
    leg_id = new_leg_id;
    radial_offset = body_radius;
    joint_names.push_back("j_coxa_");
    joint_names.push_back("j_femur_");
    joint_names.push_back("j_tibia_");
    coxa_pub = nh.advertise<std_msgs::Float64>("/"+joint_names[0]+leg_id+"_position_controller/command",1000);
    femur_pub = nh.advertise<std_msgs::Float64>("/"+joint_names[1]+leg_id+"_position_controller/command",1000);
    tibia_pub = nh.advertise<std_msgs::Float64>("/"+joint_names[2]+leg_id+"_position_controller/command",1000);
    contact_state_pub = nh.advertise<std_msgs::Bool>("/"+leg_id+"_contact/state",1000);
}

void LegKinematics::publishJointState(sensor_msgs::JointState joint_state_)
{   
    ROS_DEBUG_STREAM("Joint Angles are getting published");
    std_msgs::Float64 coxa_msg;
    std_msgs::Float64 femur_msg;
    std_msgs::Float64 tibia_msg;
    if(leg_id == "lf" || leg_id == "lm" || leg_id =="lr"){
        coxa_msg.data = -(float)joint_state_.position.at(0);
    }
    else{
        coxa_msg.data = (float)joint_state_.position.at(0);
    }
    femur_msg.data = (float)joint_state_.position.at(1);
    tibia_msg.data = (float)joint_state_.position.at(2);

    coxa_pub.publish(coxa_msg);
    femur_pub.publish(femur_msg);
    tibia_pub.publish(tibia_msg);

    // add velocity and acceleration down here later
}

sensor_msgs::JointState LegKinematics::getJointState(Vector3d& ee_pos) 
{
    Eigen::Matrix4d h;  //transformation matrix for the joint
    Eigen::Vector3d joint_angles;   //list of coxa, femur, and tibia joint angles

    double a_12 = coxa_length;
    double a_23  = femur_length;
    double a_3e = tibia_length;

    Eigen::Vector3d p_ee = Eigen::Vector3d(ee_pos.x(), ee_pos.y(), -ee_pos.z());    //position of the end effector
    joint_angles.x() = atan2(p_ee.y(),p_ee.x());    // calculate the angle of the coxa joint

    p_ee = p_ee-Vector3d(a_12,0,0); //shift over to the femur joint
    double p_2e = pow(p_ee.x(),2)+pow(p_ee.z(),2); //calculate the hypotenuse^2 from the femur joint to end effector 
    // Transformation matrix for all joints
    double alpha = atan2(-p_ee.z(),p_ee.x());   //part of the b1 angle from the LoC
    double num = (pow(a_23,2)+p_2e-pow(a_3e,2))/(2.0*a_23*sqrt(p_2e));  //LoC for femur angle
    if(num>1){  //limit to pi
        num=1;
    }
    if(num<-1){
        num=-1;
    }
    double beta = acos(num);    //other part of the triangle
    joint_angles.y() = beta+alpha; //angle of the 

    num = (pow(a_3e,2)+pow(a_23,2)-p_2e)/(2.0*a_23*a_3e);
        if(num>1){  //limit to pi
        num=1;
    }
    if(num<-1){
        num=-1;
    }
    joint_angles.z() = acos(num)-M_PI;
    
    //Enforce Limits
    ForceLegConstraints(joint_angles.x(),"coxa");
    ForceLegConstraints(joint_angles.y(),"femur");
    ForceLegConstraints(joint_angles.z(),"tibia");
    
    //return joint_angles;
    sensor_msgs::JointState joint_state_;
    std::string joint_name;
    for(size_t i=0;i<joint_names.size();++i)
    {
        joint_name = joint_names[i]+leg_id;
        joint_state_.name.push_back(joint_name.c_str());
        joint_state_.position.push_back(joint_angles[i]);
        joint_state_.velocity.push_back(0); // zero for now
        joint_state_.effort.push_back(0);   // zero for now
    }
}

Eigen::Vector3d LegKinematics::GetJointPosition(std::string joint_id) 
{

}

Eigen::Vector3d LegKinematics::GetEndEffectorPosition()
{
    double a_12 = coxa_length;
    double a_23  = femur_length;
    double a_3e = tibia_length;
    double t = 0; // coxa theta relative to body 
    double b1 = 0; // femur theta relative to coxa
    double g = 0; //gamma angle for the tibia relative to the femur
    Eigen::Matrix4d T_01;   //transformation about coxa joint in relative z-axis
    T_01 << cos(t), 0, sin(t), a_12*cos(t), 
            sin(t), 0, -cos(t), a_12*sin(t), 
            0, 1, 0, 0, 
            0, 0, 0, 1;   

    Eigen::Matrix4d T_12;   //transformation about the femur joint in relative z-axis
    T_12 << cos(b1), -sin(b1), 0, a_23*cos(b1),
            sin(b1), cos(b1), 0, a_23*sin(b1),
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4d T_23;   //transformation about the tibia joint in the relative z-axis
    T_23 << cos(g), -sin(g), 0, a_3e*cos(g),
            sin(g), cos(g), 0, a_3e*sin(g),
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    
    Eigen::Matrix4d T_03 = (T_01*T_12*T_23).eval(); //transform from the coxa joint to the tibia
    Eigen::Vector4d p_ee = Eigen::Vector4d(0, 0, 0, 1);  //position of the leg end-effector relative to the tibia joint

    Eigen::Vector4d p_0ee = (T_03*p_ee).eval();  //position of the end effector relative to the coxa joint
    Eigen::Vector3d p_0e = Eigen::Vector3d(p_0ee.x(),p_0ee.y(),p_0ee.z());
    return p_0e;
} 

void LegKinematics::ForceLegConstraints(double& q, std::string joint_id)
{
    const static double coxa_min = -60;
    const static double coxa_max = 60;

    const static double femur_min = -90;
    const static double femur_max = 90;

    const static double tibia_min = -160;
    const static double tibia_max = 160;

    static const std::map<std::string, double> min_range{   //reduce joint angles
        {"coxa", M_PI*coxa_min/180},
        {"femur", M_PI*femur_min/180},
        {"tibia", M_PI*tibia_min/180}
    };

    static const std::map<std::string, double> max_range{   //reduce joint angles
        {"coxa", M_PI*coxa_max/180},
        {"femur", M_PI*femur_max/180},
        {"tibia", M_PI*tibia_max/180}
    };

    q = q > max_range.at(joint_id) ? max_range.at(joint_id) : q;
    q = q < min_range.at(joint_id) ? min_range.at(joint_id) : q;
}

