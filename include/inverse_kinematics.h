#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include <eigen3/Eigen/Core>
namespace ardent{

    enum ArdentJointID {COXA, FEMUR, TIBIA, JointCount};

    class ArdentInverseKinematics{
        public:
            using Vector3d = Eigen::Vector3d;
            /**
             * @brief Default initialize leg lengths with values
             */
            ArdentInverseKinematics() = default;
            virtual ~ArdentInverseKinematics() = default;

            /**
             * @brief Return the joint angles to reach cartesian position
             * @param ee_pos End Effector position xyz relative to frame relative to j_c1_rf
             */ 
            Vector3d GetJointAngles(Vector3d& ee_poss);
            
            /**
             * @brief Restirct the joint angles to stay within reasonable limits
             * @param q[in/out] Current joint angle that will get adapted if out of range
             * @param joint Which joint (COXA, FEMUR, TIBIA) the joint represents
             */
            void ForceJointConstraints(double& q, ArdentJointID);
        
        private:
            Vector3d offset_z = Vector3d(0.0,0.0,0.35); //Distance from center of body to coxa
            double coxa_length = 0.1; // link length from first motor (j_coxa) to second motor (j_femur)
            double femur_length = 0.25; // link length from second motor (j_femur) to third joint (j_tibia)
            double tibia_length = 0.25; //link length from third motor (j_tibia) to the end effector
    };
}



#endif