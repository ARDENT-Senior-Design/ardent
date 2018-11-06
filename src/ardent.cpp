#include "../include/ardent.h"

namespace ardent
{
    ArdentRobot::ArdentRobot()
    {
        ArdentBodyKinematics body;
        std::vector <ArdentLegKinematics> leg(6); //initialize 6 robot legs
        leg.push_back(ArdentLegKinematics("rf", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("rm", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("rr", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("lf", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("lm", body.GetRadius()));
        leg.push_back(ArdentLegKinematics("lr", body.GetRadius()));
        //initialize the legs based on the body offset
    }

    

}