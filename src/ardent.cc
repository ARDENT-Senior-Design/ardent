#include "../include/ardent.h"

namespace ardent
{
    ArdentRobot::ArdentRobot()
    {
        leg.push_back(ArdentLegKinematics("rf", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("rm", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("rr", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("lf", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("lm", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("lr", body.GetRadius()));
        //initialize the legs based on the body offset
    }

    void ArdentRobot::PublishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos)
    {
        int leg_map = GetMappedLeg(leg_id);
        Eigen::Vector3d joint_angles= leg[leg_map].GetJointAngles(ee_pos);
        leg[leg_map].PublishJointAngles(joint_angles);
    }

    std::string ArdentRobot::GetMappedLeg(int leg_num)
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
    int ArdentRobot::GetMappedLeg(std::string leg_id)
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
    bool ArdentRobot::CheckStability()
    {
        return true;
    }

}