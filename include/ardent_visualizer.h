#ifndef ARDENT_Visualizer_H_
#define ARDENT_Visualizer_H_

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include "leg_kinematics.h"
namespace ardent{

    class UrdfVisualizer{
        public:
        using urdfNameToJointAngle = std::map<std::string, float>;

            UrdfVisualizer(const std::string& name,
                           const std::vector<std::string>& joint_names_,
                           const std::string& base_joint_,
                           const std::string& rviz_fixed_frame_,
                           const std::string& robot_state_topic);
            virtual ~UrdfVisualizer() = default;
        private:
            ros::Subscriber state_sub;
            tf::TransformBroadcaster tf_broadcaser;
            std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_pub;

            void RobotStateCallback(const ArdentLegKinematics& leg);
            urdfNameToJointAngle AssignAngleToURDFJoint(sensor_msgs::JointState& msg);
            geometry_msgs::TransformStamped GetRosBase(ros::Time& stamp, geometry_msgs::Pose& msg);

            std::vector<std::string> joint_names;
            std::string base_joint;
            std::string state_msg_name;
            std::string rviz_fixed_frame;

    };
}

#endif