#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <eigen3/Eigen/Dense>
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
            ArdentLegKinematics(ArdentLegID id);
            virtual ~ArdentLegKinematics() = default;

            /**
             * @brief Return the joint angles to reach cartesian position in a vector array relative to robot center. In order, the vector array of each angle is: Coxa, femur, tibia
             * @param ee_pos End Effector position xyz relative to frame relative to j_c1_rf
             */ 
            Vector3d GetJointAngles(Vector3d& ee_pos);
            
            /**
             * @brief Return the cartesian position
             * @param id Joint id for the position
             */
            Vector3d GetJointPosition(ArdentJointID id);

            /**
             * @brief Returns the cartesian position of an end effector
             * @param id Leg id for which specific leg of the six
             */
            Vector3d GetEndEffectorPosition();

            /**
             * @brief Restirct the joint angles to stay within reasonable limits
             * @param q[in/out] Current joint angle that will get adapted if out of range
             * @param joint Which joint (COXA, FEMUR, TIBIA) the joint represents
             */
            void ForceLegConstraints(double& q, ArdentLegID id);
            
            /**
             * @brief Returns the 0-angle of the leg relative to the IMU direction
             * @param id Leg
             */
            double LegAngle();
        
        private:
            Vector3d offset_z = Vector3d(0.1,0.173,0.0); //Distance from center of body to coxa
            double coxa_length = 0.1; // link length from first motor (j_coxa) to second motor (j_femur)
            double femur_length = 0.25; // link length from second motor (j_femur) to third joint (j_tibia)
            double tibia_length = 0.25; //link length from third motor (j_tibia) to the end effector
            ArdentLegID leg_id;
    };
}



#endif