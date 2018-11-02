#include <../include/kinematics.h>


namespace ardent{

        ArdentLegKinematics::ArdentLegKinematics(ArdentLegID id)
        {
            leg_id = id;
        }

        ArdentLegKinematics::Vector3d ArdentLegKinematics::GetJointAngles(Vector3d& ee_pos) 
        {
            Eigen::Matrix4d h;  //transformation matrix for the joint
            Vector3d qr = ee_pos;
            
            // Transformation matrix for all joints
            
            //Enforce Limits
        }

        ArdentLegKinematics::Vector3d ArdentLegKinematics::GetJointPosition(ArdentJointID id) 
        {
            
        }

        ArdentLegKinematics::Vector3d ArdentLegKinematics::GetEndEffectorPosition()
        {
            
        } 

        void ArdentLegKinematics::ForceLegConstraints(double& q, ArdentLegID id)
        {
            
        }

        double ArdentLegKinematics::LegAngle()
        {
            switch(leg_id)
            {
                case RF:
                    return 1.0472;
                break;
                case RM:
                    return 0;
                break;
                case RR:
                    return -1.0472;
                break;
                case LF:
                    return 2.0944;
                break;
                case LM:
                    return 3.14;
                break;
                case LR:
                    return 4.18879;
                break;
            }
        
        }

}