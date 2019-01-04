#ifndef ROBOT_H_
#define ROBOT_H_

#include <ros/ros.h>
#include "leg_kinematics.h"
#include "body_kinematics.h"
#include <string>
#include <hardware_interface/hardware_interface.h>
#include "joint.h"
namespace ardent_model {

    class Robot{
        public:
            /** 
             * Supporting Library
             * @brief Creates a new ARDENT robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            Robot(std::vector<std::string> legs_);
            ~Robot(){} //= default;
            
            void PublishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos);

            std::string GetMappedLeg(int leg_num);
            int GetMappedLeg(std::string leg_id);

            bool CheckStability();

        private: 

        BodyKinematics body;
        std::vector<LegKinematics> leg; //initialize 6 robot legs
        int num_legs;
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
        /// constructor
        RobotState(Robot *model);
        /// The robot model containing the transmissions, urdf robot model, and hardware interface
        Robot *model_;
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
        // std::vector<std::vector<pr2_hardware_interface::Actuator*> > transmissions_in_;

        /**
         * Each transmission refers to the actuators and joints it connects by name.
         * Since name lookup is slow, for each transmission in the robot model we
         * cache pointers to the actuators and joints that it connects.
         **/
        // std::vector<std::vector<pr2_mechanism_model::JointState*> > transmissions_out_;

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


        std::map<std::string, JointState*> joint_states_map_;

    };


}



#endif