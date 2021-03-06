#ifndef ROBOT_H_
#define ROBOT_H_

#include <ros/ros.h>
#include "leg_kinematics.h"
#include "body_kinematics.h"
#include <urdf/model.h>
#include <map>
#include "hardware_interface.h"
#include <string>
#include <hardware_interface/hardware_interface.h>
#include <sensor_msgs/JointState.h>
#include "transmission.h"
#include "joint.h"
#include <pluginlib/class_loader.h>
#include <tinyxml.h>

namespace ardent_model {

    class Robot{
        private: 
            boost::shared_ptr<pluginlib::ClassLoader<ardent_model::Transmission> > transmission_loader_;
        public:
            BodyKinematics body_;
            std::vector<LegKinematics> legs_;
            int num_legs_;

            /// a pointer to the ardent hardware interface. 
            ardent_hardware_interface::HardwareInterface* hw_;
            urdf::Model robot_model_;
            std::vector<boost::shared_ptr<Transmission> > transmissions_;
       
            /** 
             * Supporting Library
             * @brief Creates a new robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            Robot(ardent_hardware_interface::HardwareInterface *hw);
            ~Robot(){}

            /// Initialize the robot model form xml
            bool initXml(TiXmlElement *root);
            
            int getTransmissionIndex(const std::string &name) const;

            /// get an actuator pointer based on the actuator name. Returns NULL on failure
            ardent_hardware_interface::Actuator* getActuator(const std::string &name) const;

            /// get a transmission pointer based on the transmission name. Returns NULL on failure
            boost::shared_ptr<ardent_model::Transmission> getTransmission(const std::string &name) const;

            /// Get the time when the current controller cycle was started
            ros::Time getTime();

            void publishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos); //TODO: Change to include pose, vel

            std::string getMappedLeg(int leg_num);
            int getMappedLeg(std::string leg_id);

            bool checkStability();
    }; 

    /** \brief This class provides the controllers with an interface to the robot state
     *
     * Most controllers that need the robot state should use the joint states, to get
     * access to the joint position/velocity/effort, and to command the effort a joint
     * should apply. Controllers can get access to the hard realtime clock through getTime()
     *
     * Some specialized controllers (such as the calibration controllers) can get access
     * to actuator states, and transmission states.
     */
    class RobotState : public hardware_interface::HardwareInterface
    {
        public:

        std::map<std::string, JointState*> joint_states_map_;

        // time since starting the robot 
        ros::Time current_time_;
        /// constructor
        RobotState(Robot *model);
        /// The robot model containing the transmissions, urdf robot model, and hardware interface
        Robot *model_;
        ros::Time getTime();
        /// The vector of joint states, in no particular order
        std::vector<JointState> joint_states_;
        /// Get a joint state by name
        JointState *getJointState(const std::string &name);
        /// Get a const joint state by name
        const JointState *getJointState(const std::string &name) const;

        /// Get the time when the current controller cycle was started
        // ros::Time getTime() {return model_->getTime();};
        /**
         * Each transmission refers to the actuators and joints it connects by name.
         * Since name lookup is slow, for each transmission in the robot model we
         * cache pointers to the actuators and joints that it connects.
         **/
        std::vector<std::vector<ardent_hardware_interface::Actuator*> > transmissions_in_;

        /**
         * Each transmission refers to the actuators and joints it connects by name.
         * Since name lookup is slow, for each transmission in the robot model we
         * cache pointers to the actuators and joints that it connects.
         **/
        std::vector<std::vector<ardent_model::JointState*> > transmissions_out_;

        /// Propagete the actuator positions, through the transmissions, to the joint positions
        void propagateActuatorPositionToJointPosition();
        /// Propagete the joint positions, through the transmissions, to the actuator positions
        void propagateJointPositionToActuatorPosition();

        /// Propagete the joint efforts, through the transmissions, to the actuator efforts
        void propagateJointEffortToActuatorEffort();
        /// Propagete the actuator efforts, through the transmissions, to the joint efforts
        void propagateActuatorEffortToJointEffort();

        /// Modify the commanded_effort_ of each joint state so that the joint limits are satisfied
        void enforceSafety();

        /// Checks if one (or more) of the motors are halted.
        bool isHalted();

        /// Set the commanded_effort_ of each joint state to zero
        void zeroCommands();

    };


}



#endif