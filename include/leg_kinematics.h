#ifndef LEG_KINEMATICS_H_
#define LEG_KINEMATICS_H_

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <cmath>
#include <map>
namespace ardent{

    typedef enum ArdentJointID {COXA, FEMUR, TIBIA, JointCount};    // specify which quaternion point to look at
    typedef enum ArdentLegID {RF, RM, RR, LF, LM, LR};  //specify which leg being coordinated

    class ArdentLegKinematics{
        public:
            using Vector3d = Eigen::Vector3d;
            /**
             * @brief Default initialize leg lengths with values
             */
            ArdentLegKinematics(ArdentLegID id, double radial_offset);
            virtual ~ArdentLegKinematics() = default;

            /**
             * @brief Gets the leg angles based on an end-effector position. 
             * @param ee_pos End Effector position xyz relative to frame relative to j_c1_rf
             * @return Get the joint angles to reach cartesian position in a vector array relative to robot center. In order: the vector array of each angle is: Coxa, femur, tibia
             */ 
            Vector3d GetJointAngles(Vector3d& ee_pos);
            
            /**
             * @brief Get the position of a specific joint in the leg
             * @param id Joint id for the position
             * @return The cartesian position of the joint relative to the coxa, where 
             */
            Vector3d GetJointPosition(ArdentJointID id);

            /**
             * @brief Returns the cartesian position of an end effector
             * @param id Leg id for which specific leg of the six
             * @return The Cartesian position of the leg end-effector relative to the coxa
             */
            Vector3d GetEndEffectorPosition();

            /**
             * @brief Restirct the joint angles to stay within reasonable limits
             * @param q[in/out] Current joint angle that will get adapted if out of range
             * @param joint Which joint (COXA, FEMUR, TIBIA) the joint represents 
             */
            void ForceLegConstraints(double& q, ArdentJointID id);
            
           
        
        private:
            double radial_offset; //Distance from center of body to coxa
            double coxa_length = 0.1; // link length from first motor (j_coxa) to second motor (j_femur)
            double femur_length = 0.25; // link length from second motor (j_femur) to third joint (j_tibia)
            double tibia_length = 0.25; //link length from third motor (j_tibia) to the end effector

            // Angle should be kept track of by encoders, I will try to keep track of them here
            ArdentLegID leg_id;
    };
}



#endif