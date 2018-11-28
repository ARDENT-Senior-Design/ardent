#include "../include/leg_kinematics.h"


namespace ardent{

        ArdentLegKinematics::ArdentLegKinematics(std::string new_leg_id, double body_radius)
        {
            leg_id = new_leg_id;
            radial_offset = body_radius;
            femur_pub = nh.advertise<std_msgs::Float64>("/j_femur_"+leg_id+"_position_controller/command",1000);
            tibia_pub = nh.advertise<std_msgs::Float64>("/j_tibia_"+leg_id+"_position_controller/command",1000);
            coxa_pub = nh.advertise<std_msgs::Float64>("/j_coxa_"+leg_id+"_position_controller/command",1000);
            contact_state_pub = nh.advertise<std_msgs::Bool>("/"+leg_id+"_contact/state",1000);
        }

        void ArdentLegKinematics::PublishJointAngles(Vector3d& j_pos)
        {   
            ROS_DEBUG_STREAM("Joint Angles are getting published");
            std_msgs::Float64 coxa_msg;
            std_msgs::Float64 femur_msg;
            std_msgs::Float64 tibia_msg;
            if(leg_id == "lf" || leg_id == "lm" || leg_id =="lr"){
                coxa_msg.data = -(float)j_pos.x();
            }
            else{
                coxa_msg.data = (float)j_pos.x();
            }
            femur_msg.data = (float)j_pos.y();
            tibia_msg.data = (float)j_pos.z();

            coxa_pub.publish(coxa_msg);
            femur_pub.publish(femur_msg);
            tibia_pub.publish(tibia_msg);
        }

        Eigen::Vector3d ArdentLegKinematics::GetJointAngles(Vector3d& ee_pos) 
        {
            Eigen::Matrix4d h;  //transformation matrix for the joint
            Eigen::Vector3d joint_angles;   //list of coxa, femur, and tibia joint angles

            double a_12 = coxa_length;
            double a_23  = femur_length;
            double a_3e = tibia_length;

            Eigen::Vector3d p_ee = Eigen::Vector3d(ee_pos.x(), ee_pos.y(), -ee_pos.z());    //position of the end effector
            joint_angles.x() = atan2(p_ee.y(),p_ee.x());    // calculate the angle of the coxa joint

            p_ee = p_ee-Vector3d(a_12,0,0); //shift over to the femur joint
            double p_2e = pow(p_ee.x(),2)+pow(p_ee.z(),2); //calculate the hypotenuse^2 from the femur joint to end effector 
            // Transformation matrix for all joints
            double alpha = atan2(-p_ee.z(),p_ee.x());   //part of the b1 angle from the LoC
            double num = (pow(a_23,2)+p_2e-pow(a_3e,2))/(2.0*a_23*sqrt(p_2e));  //LoC for femur angle
            if(num>1){  //limit to pi
                num=1;
            }
            if(num<-1){
                num=-1;
            }
            double beta = acos(num);    //other part of the triangle
            joint_angles.y() = beta+alpha; //angle of the 

            num = (pow(a_3e,2)+pow(a_23,2)-p_2e)/(2.0*a_23*a_3e);
             if(num>1){  //limit to pi
                num=1;
            }
            if(num<-1){
                num=-1;
            }
            joint_angles.z() = acos(num)-M_PI;
            
            //Enforce Limits
            ForceLegConstraints(joint_angles.x(),"coxa");
            ForceLegConstraints(joint_angles.y(),"femur");
            ForceLegConstraints(joint_angles.z(),"tibia");
            
            return joint_angles;
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

            Eigen::Vector4d p_0e4 = (T_03*p_3e).eval();  //position of the end effector relative to the coxa joint
            Eigen::Vector3d p_0e = Eigen::Vector3d(p_0e4.x(),p_0e4.y(),p_0e4.z());
            return p_0e;
        } 

        void ArdentLegKinematics::ForceLegConstraints(double& q, std::string joint_id)
        {
            const static double coxa_min = -60;
            const static double coxa_max = 60;

            const static double femur_min = -90;
            const static double femur_max = 90;

            const static double tibia_min = -160;
            const static double tibia_max = 160;

            static const std::map<std::string, double> min_range{   //reduce joint angles
                {"coxa", M_PI*coxa_min/180},
                {"femur", M_PI*femur_min/180},
                {"tibia", M_PI*tibia_min/180}
            };

            static const std::map<std::string, double> max_range{   //reduce joint angles
                {"coxa", M_PI*coxa_max/180},
                {"femur", M_PI*femur_max/180},
                {"tibia", M_PI*tibia_max/180}
            };

            q = q > max_range.at(joint_id) ? max_range.at(joint_id) : q;
            q = q < min_range.at(joint_id) ? min_range.at(joint_id) : q;
        }

}