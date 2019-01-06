#ifndef BODY_KINEMATICS_H_
#define BODY_KINEMATICS_H_

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <string>

namespace ardent_model {

    class BodyKinematics : public Transmission
    {
        #define BODY_THICKNESS 2
        public:
            typedef Eigen::Vector3d Vector3d;
            typedef Eigen::Matrix4d Matrix4d;

            /**
             * Supporting Library
             * @brief Constructor for the body frame of the robot
             */
            BodyKinematics(); //= default;
            // ~BodyKinematics(); //= default;
            
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
            Vector3d rpy;   //roll, pitch, and yaw of the robot body
            Vector3d body_pose;
            double leg_radius;
            
    };
}
#endif