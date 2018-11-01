#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <eigen3/Eigen/Dense>

namespace ardent{

    enum ArdentJointID {COXA, FEMUR, TIBIA, EE, JointCount};

    class ArdentKinematics{
        public:
            using Vector3d = Eigen::Vector3d;
            /**
             * @brief Default initialize leg lengths with values
             */
            ArdentKinematics() = default;
            virtual ~ArdentKinematics() = default;

            /**
             * @brief Return the joint angles to reach cartesian position in a vector array. In order, the vector array of each angle is: Coxa, femur, tibia
             * @param ee_pos End Effector position xyz relative to frame relative to j_c1_rf
             */ 
            Vector3d GetJointAngles(Vector3d& ee_pos);
            
            /**
             * @brief Return the cartesian position
             * @param id Joint id for the position
             */
            Vector3d GetJointPosition(ArdentJointID id);


            /**
             * @brief Restirct the joint angles to stay within reasonable limits
             * @param q[in/out] Current joint angle that will get adapted if out of range
             * @param joint Which joint (COXA, FEMUR, TIBIA) the joint represents
             */
            void ForceJointConstraints(double& q, ArdentJointID id);


        
        private:
            Vector3d offset_z = Vector3d(1.53,0.85,0.0); //Distance from center of body to coxa
            double coxa_length = 0.1; // link length from first motor (j_coxa) to second motor (j_femur)
            double femur_length = 0.25; // link length from second motor (j_femur) to third joint (j_tibia)
            double tibia_length = 0.25; //link length from third motor (j_tibia) to the end effector
    };
}



#endif