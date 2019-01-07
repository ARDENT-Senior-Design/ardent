#include "../include/ardent/robot.h"

using namespace ardent_model;

Robot::Robot(std::vector<std::string> legs_)
{
    num_legs = legs_.size();
    
    for(int i=0;i<legs_.size();i++){
        legs.push_back(LegKinematics(legs_[i], body.GetRadius()));
    }
    //initialize the legs based on the body offset
}

ros::Time Robot::getTime()
{
    return current_time;
}

void Robot::publishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos)
{
    int leg_map = getMappedLeg(leg_id);
    sensor_msgs::JointState joint_state= legs[leg_map].getJointState(ee_pos);
    legs[leg_map].publishJointState(joint_state);
}

std::string Robot::getMappedLeg(int leg_num)
{
    static const std::map<int,std::string> leg_map{
        {0, "rf"},
        {1,"rm"},
        {2, "rr"},
        {3, "lf"},
        {4, "lm"},
        {5, "lr"}
    };
    return leg_map.at(leg_num);
}
int Robot::getMappedLeg(std::string leg_id)
{
    static const std::map<std::string, int> leg_map{
        {"rf", 0},
        {"rm", 1},
        {"rr", 2},
        {"lf", 3},
        {"lm", 4},
        {"lr", 5}
    };return leg_map.at(leg_id);

}
bool Robot::checkStability()
{
    std::vector<float> contact_legs;
    for(int i=0;i<num_legs;i++){
        //if(GetMappedLeg(i))
    }
}

