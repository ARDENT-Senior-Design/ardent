#ifndef ARDENT_H_
#define ARDENT_H_

#include <ros/ros.h>
#include "leg_kinematics.h"
#include "body_kinematics.h"
#include <string>
namespace ardent{

    class ArdentRobot{
        public:
            /** 
             * @brief Creates a new ARDENT robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            ArdentRobot(std::vector<std::string> legs_);
            ~ArdentRobot() = default;
            
            void PublishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos);

            std::string GetMappedLeg(int leg_num);
            int GetMappedLeg(std::string leg_id);

            bool CheckStability();

        private: 

        ArdentBodyKinematics body;
        std::vector<ArdentLegKinematics> leg; //initialize 6 robot legs
        int num_legs;
    };

}

#endif