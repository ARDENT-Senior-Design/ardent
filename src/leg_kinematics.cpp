#include <../include/leg_kinematics.h>


namespace ardent{

        ArdentLegKinematics::ArdentLegKinematics(std::string new_leg_id, double body_radius)
        {
            leg_id = new_leg_id;
            radial_offset = body_radius;
            ros::Publisher femur_rf_pub = nh.advertise<std_msgs::Float64>("/ardent/j_femur_"+leg_id+"_position_controller/command",1000);
            ros::Publisher tibia_rf_pub = nh.advertise<std_msgs::Float64>("/ardent/j_tibia_"+leg_id+"_position_controller/command",1000);
            ros::Publisher coxa_rf_pub = nh.advertise<std_msgs::Float64>("/ardent/j_coxa_"+leg_id+"_position_controller/command",1000);
        }

        Eigen::Vector3d ArdentLegKinematics::GetJointAngles(Vector3d& ee_pos) 
        {
            Eigen::Matrix4d h;  //transformation matrix for the joint
            Eigen::Vector3d joint_angles;   //list of coxa, femur, and tibia joints

            double a_12 = coxa_length;
            double a_23  = femur_length;
            double a_3e = tibia_length;

            Eigen::Vector3d p_ee = (ee_pos.x, ee_pos.y, -ee_pos.z);    //position of the end effector
            joint_angles.x = atan2(p_ee.y,p_ee.x);    // calculate the angle of the coxa joint

            p_ee = p_ee-Vector3d(a_12,0,0); //shift over to the femur joint
            double p_2e = pow(p_ee.x,2)+pow(p_ee.z,2); //calculate the hypotenuse^2 from the femur joint to end effector 
            // Transformation matrix for all joints
            double alpha = atan2(-p_ee.z,p_ee.x);   //part of the b1 angle from the LoC
            double num = (pow(a_23,2)+p_2e-pow(a_3e,2))/(2.0*a_23*sqrt(p_2e));  //LoC for femur angle
            if(num>1){  //limit to pi
                num=1;
            }
            if(num<-1){
                num=-1;
            }
            double beta = acos(num);    //other part of the triangle
            joint_angles.y = beta+alpha; //angle of the 

            num = (pow(a_3e,2)+pow(a_23,2)-p_2e)/(2.0*a_23*a_3e);
             if(num>1){  //limit to pi
                num=1;
            }
            if(num<-1){
                num=-1;
            }
            joint_angles.z = acos(num)-M_PI;
            return joint_angles;
            //Enforce Limits
        }

        Eigen::Vector3d ArdentLegKinematics::GetJointPosition(std::string joint_id) 
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

        void ArdentLegKinematics::ForceLegConstraints(double& q, std::string joint_id)
        {
            
        }

}