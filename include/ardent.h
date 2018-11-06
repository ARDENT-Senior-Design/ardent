#ifndef ARDENT_H_
#define ARDENT_H_

#include <body_kinematics.h>
#include <leg_kinematics.h>
#include <ros/ros.h>

namespace ardent{

    class ArdentRobot{
        public:
            /** 
             * @brief Creates a new ARDENT robot
             */
            ArdentRobot() = default;
            ~ArdentRobot() = default;
            
        private: 

    };

}

#endif