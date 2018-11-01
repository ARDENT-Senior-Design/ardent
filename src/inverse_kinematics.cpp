#include <../include/inverse_kinematics.h>


namespace ardent{
    
        ArdentKinematics::Vector3d ArdentKinematics::GetJointAngles(Vector3d& ee_pos)
        {
            Eigen::Matrix4d h;  //transformation matrix for the joint
            Vector3d qr = ee_pos;
            
            // Transformation matrix for all joints
            
            //Enforce Limits
        }

        ArdentKinematics::Vector3d ArdentKinematics::GetJointPosition(ArdentJointID& q)
        {

        }

        ArdentKinematics::Vector3d ArdentKinematics::ForceJointConstraints(double& qq, ArdentJointID id)
        {
            
        }

}