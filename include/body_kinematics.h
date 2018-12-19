#ifndef BODY_KINEMATICS_H_
#define BODY_KINEMATICS_H_

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <string>

namespace ardent
{
    class ArdentBodyKinematics
    {
        #define BODY_THICKNESS 2
        public:
            using Vector3d = Eigen::Vector3d;
            using Matrix4d = Eigen::Matrix4d;

            /**
             * Supporting Library
             * @brief Constructor for the body frame of the robot
             */
            ArdentBodyKinematics() = default;
            virtual ~ArdentBodyKinematics() = default;
            
            /**
             * @brief The transform to the coxa joint of a leg based on the orientation and size of the body
             * @return The transform from the centroid of the body to the leg location based on the RPY of the robot body and relative leg coordinate
             */
            Matrix4d GetLegPosition(std::string leg_id);

             /**
             * @brief Returns the 0-angle of the leg relative to the IMU direction
             * @return The angular offset of the leg regular to the X-axis
             */
            double GetLegAngleOffset(std::string leg_id);

            /** 
             * @brief Get the radius for the hexagon body
             * @return The radius in [meters] of the body
             */
            double GetRadius();

            

        private:
            Vector3d rpy = Vector3d(0, 0, 0);   //roll, pitch, and yaw of the robot body
            Vector3d body_pose = Vector3d(0, 0, BODY_THICKNESS);
            double leg_radius = 0.2;
            
    };


}

#endif