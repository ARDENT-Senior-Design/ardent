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
        // static const std::map<std::string, int> leg_map{
        //     {"rf", 0},
        //     {"rm", 1},
        //     {"rr", 2},
        //     {"lf", 3},
        //     {"lm", 4},
        //     {"lr", 5}
        // };
        // Eigen::Vector3d joint_angles= leg[leg_map.at(leg_id)].GetJointAngles(ee_pos);
        // leg[leg_map.at(leg_id)].PublishJointAngles(joint_angles);
    }

}