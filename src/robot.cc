#include "../include/robot.h"

namespace ardent
{
    Robot::Robot(std::vector<std::string> legs_)
    {
        num_legs = legs_.size();
        for(int i=0;i<legs_.size();i++){
            leg.push_back(ArdentLegKinematics(legs_[i], body.GetRadius()));
        }
        //initialize the legs based on the body offset
    }

    void Robot::PublishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos)
    {
        int leg_map = GetMappedLeg(leg_id);
        Eigen::Vector3d joint_angles= leg[leg_map].GetJointAngles(ee_pos);
        leg[leg_map].PublishJointAngles(joint_angles);
    }

    std::string Robot::GetMappedLeg(int leg_num)
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
    int Robot::GetMappedLeg(std::string leg_id)
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
    bool Robot::CheckStability()
    {
        std::vector<float> contact_legs;
        for(int i=0;i<num_legs;i++){
            //if(GetMappedLeg(i))
        }
    }

}