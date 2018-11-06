#include <../include/leg_kinematics.h>


namespace ardent{

        ArdentLegKinematics::ArdentLegKinematics(ArdentLegID id, double body_radius)
        {
            leg_id = id;
            radial_offset = body_radius;
        }

        Eigen::Vector3d ArdentLegKinematics::GetJointAngles(Vector3d& ee_pos) 
        {
            Eigen::Matrix4d h;  //transformation matrix for the joint
            Vector3d p_ee = (ee_pos.x, ee_pos.y, -ee_pos.z);
            
            // Transformation matrix for all joints
            
            //Enforce Limits
        }

        Eigen::Vector3d ArdentLegKinematics::GetJointPosition(ArdentJointID id) 
        {

        }

        Eigen::Vector3d ArdentLegKinematics::GetEndEffectorPosition()
        {
            double a_12 = coxa_length;
            double a_23  = femur_length;
            double a_3e = tibia_length;
            double t = 0; // coxa theta relative to body 
            double b1 = 0; // femur theta relative to coxa
            double g = 0; //gamma angle for the tibia relative to the femur
            Eigen::Matrix4d T_01;   //transformation about coxa joint in relative z-axis
            T_01 << cos(t), -sin(t), 0, 0, sin(t), cos(t), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;   

            Eigen::Matrix4d T_12;   //transformation about the femur joint in relative z-axis
            T_12 << cos(b1), -sin(b1), 0, a_12, 0, 0, -1, 0, sin(b1), cos(b1), 0, 0, 0, 0, 0, 1;

            Eigen::Matrix4d T_23;   //transformation about the tibia joint in the relative z-axis
            T_23 << cos(g), -sin(g), 0, a_23, sin(g), cos(g), 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
            
            Eigen::Vector4d p_3e = Eigen::Vector4d(a_3e, 0, 0, 1);  //position of the leg end-effector relative to the tibia joint
            
            Eigen::Matrix4d T_03 = (T_01*T_12*T_23).eval(); //transform from the coxa joint to the tibia

            Eigen::Vector4d p_0e = (T_03*p_3e).eval();  //position of the end effector relative to the coxa joint
            return p_0e;
        } 

        void ArdentLegKinematics::ForceLegConstraints(double& q, ArdentJointID id)
        {
            
        }

}