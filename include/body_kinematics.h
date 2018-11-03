#ifndef BODY_KINEMATICS_H_
#define BODY_KINEMATICS_H_

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

namespace ardent
{
    class ArdentBodyKinematics
    {
        #define BODY_THICKNESS 2
        public:
            using Vector3d = Eigen::Vector3d;

            ArdentBodyKinematics() = default;
            virtual ~ArdentBodyKinematics() = default;

            double GetRadius();

        private:
            Vector3d rpy = Vector3d(0, 0, 0);   //roll, pitch, and yaw of the robot body
            Vector3d body_pose = Vector3d(0, 0, BODY_THICKNESS);
            double body_radius = 0.2;
    };


}

#endif