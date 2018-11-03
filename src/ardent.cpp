#include "../include/ardent.h"

namespace ardent
{
    ArdentRobot::ArdentRobot()
    {
        ArdentBodyKinematics body;
        std::vector <ArdentLegKinematics> leg(6); //initialize 6 robot legs
        leg.push_back(ArdentLegKinematics(RF, body.GetRadius()));
        leg.push_back(ArdentLegKinematics(RM, body.GetRadius()));
        leg.push_back(ArdentLegKinematics(RR, body.GetRadius()));
        leg.push_back(ArdentLegKinematics(LF, body.GetRadius()));
        leg.push_back(ArdentLegKinematics(LM, body.GetRadius()));
        leg.push_back(ArdentLegKinematics(LR, body.GetRadius()));
        //initialize the legs based on the body offset
    }


}