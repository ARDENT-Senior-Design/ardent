#include "../include/ardent/body_kinematics.h"

using namespace ardent_model;

BodyKinematics::BodyKinematics() 
    : rpy(Vector3d(0, 0, 0)), body_pose(Vector3d(0, 0, BODY_THICKNESS)), leg_radius(0.2)
{
}


Eigen::Matrix4d BodyKinematics::GetLegPosition(std::string leg_id)
{
    double t_d = BodyKinematics::GetLegAngleOffset(leg_id);
    double t_y = 0; // yaw of the robot
    double t_r = 0; // roll of the robot
    double t_p = 0; // pitch of the robot

    Eigen::Matrix4d Tz; // translation about the robot z-axis
    Tz << cos(t_d+t_y), -sin(t_d+t_y), 0, leg_radius*cos(t_d), sin(t_d+t_y), cos(t_d+t_y), 0, leg_radius*sin(t_d), 0, 0, 1, 0, 0, 0, 0, 1;

    Eigen::Matrix4d Tx;
    Tx << 1, 0, 0, BodyKinematics::GetRadius(), 0, cos(t_r), -sin(t_r), 0, 0, sin(t_r), cos(t_r), 0, 0, 0, 0, 1;

    Eigen::Matrix4d Ty;
    Ty << cos(t_p), 0, sin(t_p), 0, 0, 1, 0, 0, -sin(t_p), 0, cos(t_p), 0, 0, 0, 0, 1;

    return (Tx*Ty*Tz).eval();
}

double BodyKinematics::GetLegAngleOffset(std::string leg_id)
{
    if(leg_id == "rf"){
        return 1.0472;
    }
    else if(leg_id == "rm"){
        return 0;
    }
    else if(leg_id == "rr"){
        return -1.0472;
    }
    else if(leg_id == "lf"){
        return 2.0944;
    }
    else if(leg_id == "lm"){
        return 3.14;
    }
    else if(leg_id == "lr"){
        return 4.128879;
    }
    else{
        return 0;
    }
    
}

double BodyKinematics::GetRadius()
{
    return leg_radius;
}
