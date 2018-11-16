#include "../include/ardent_visualizer.h"

namespace ardent{

    UrdfVisualizer::UrdfVisualizer(const std::string& name,
                    const std::vector<std::string>& joint_names_,
                    const std::string& base_joint_,
                    const std::string& rviz_fixed_frame_,
                    const std::string& robot_state_topic)
    {
    joint_names = joint_names_;
    base_joint  = base_joint_;
    rviz_fixed_frame   = rviz_fixed_frame_;

    ros::NodeHandle nh;
    state_sub = nh.subscribe(robot_state_topic, 1, &UrdfVisualizer::RobotStateCallback, this);
    ROS_INFO("Subscribed to: %s", state_sub.getTopic().c_str());

    // Load model from file
    // KDL::Tree my_kdl_tree;
    // urdf::Model my_urdf_model;
    // bool model_ok  = my_urdf_model.initParam(name);
    // if(!model_ok)
    // {
    //     ROS_ERROR("Invalid URDF File");
    //     exit(EXIT_FAILURE);
    // }
    // ROS_INFO("URDF successfully parsed");
    // // kdl_parser::treeFromUrdfModel(my_urdf_model, my_kdl_tree);
    // ROS_INFO("Robot tree is ready");

    // robot_pub = std::make_shared<robot_state_publisher::RobotStatePublisher>(my_kdl_tree);
    }

    void UrdfVisualizer::RobotStateCallback(const ArdentLegKinematics& leg)
    {
        // auto joint_positions = AssignAngleToURDFJoint(leg.joint_state);
    }
    UrdfVisualizer::urdfNameToJointAngle UrdfVisualizer::AssignAngleToURDFJoint(sensor_msgs::JointState &msg)
    {
        urdfNameToJointAngle q;

        for (int i=0; i<msg.position.size(); ++i)
            q[joint_names.at(i)] = msg.position.at(i);

        return q;
    }
}   
