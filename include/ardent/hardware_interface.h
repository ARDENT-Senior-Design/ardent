#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <eigen3/Eigen/Dense>

namespace ardent_hardware_interface
{
    class ActuatorState
    {
        private:
            /**
             * The time at which actuator state was measured, relative to the time the ethercat process was started.
             * Timestamp value is not synchronised with wall time and may be different for different actuators.
             * For Willow Garage motor controllers, timestamp is made when actuator data is sampled.
             * sample_timestamp_ will provide better accuracy than ros::Time::now() or robot->getTime()
             * when using a time difference in calculations based on actuator variables.
             */
            ros::Duration sample_timestamp;
            
            /** The time at which this actuator state was measured (in seconds).
             * This value should be same as sample_timestamp_.toSec() for Willow Garage devices.
             * The timestamp_ variable is being kept around for backwards compatibility, new controllers
             * should use sample_timestamp_ instead.
             */
            double timestamp;

            int device_id;  // position in the USB chain
            int encoder_count;
            double position;
            double velocity; // velocity of the joint based on encoder speed

            bool calibrate_reading;
            bool calibrate_rising_edge_valid;
            bool calibrate_falling_edge_valid;
            double prev_calibrate_rising_edge;
            double prev_calibrate_falling_edge;
            
            bool is_enabled;
            bool is_halted;

            double prev_current_command;    // current computed based on effort of Actuator
            double prev_current_executed;   // actual current after enforced limits
            double prev_current_measured;   // measured current

            double prev_effort_command;     // torque requested in actuator from 
            double prev_effort_executed;    // torque applied after limits enforced
            double prev_effort_measured;    // last measured torque

            double max_effort;
            double motor_voltage;
            double encoder_error_count; // number of invalid encoder transmissions
            double zero_offset;
        public:
            ActuatorState() :
                timestamp(0),
                device_id(0),
                encoder_count(0),
                position(0),
                velocity(0),
                calibrate_reading(0),
                calibrate_rising_edge_valid(0),
                calibrate_falling_edge_valid(0),
                prev_calibrate_rising_edge(0),
                prev_calibrate_falling_edge(0),
                is_enabled(0),
                is_halted(0),
                prev_current_command(0),
                prev_current_executed(0),
                prev_current_measured(0),
                prev_effort_command(0),
                prev_effort_executed(0),
                prev_effort_measured(0),
                motor_voltage(0),
                encoder_error_count(0),
                zero_offset(0)
            {}
    };

    class ActuatorCommand
    {
        private:
            bool enable;
            bool effort;
        public:
            ActuatorCommand() :
                enable(0), effort(0)
            {
            }
    };  

    class Actuator
    {
        private:
            std::string name;
            ActuatorState state;
            ActuatorCommand command;
        public:
            Actuator(){};
            Actuator(std::string name_) : name(name_){}
    };

    typedef std::map<std::string, Actuator*> ActuatorMap;
    // typedef std::map<std::string, PressureSensor*> PressureSensorMap;
    // typedef std::map<std::string, Accelerometer*> AccelerometerMap;
    // typedef std::map<std::string, ForceTorque*> ForceTorqueMap;
    // typedef std::map<std::string, DigitalOut*> DigitalOutMap;
    // typedef std::map<std::string, Projector*> ProjectorMap;
    // typedef std::map<std::string, AnalogIn*> AnalogInMap;
    // typedef std::map<std::string, CustomHW*> CustomHWMap;

    /*!
    * \class HardwareInterface
    * The HardwareInterface class provides access to the PR2 hardware
    * components that are controlled via EtherCAT.  These components include:
    *  - Actuators
    *  - Finger-tip Pressure Sensors
    *  - Accelerometers
    *  - Force/Torque Sensors
    *  - Digital I/Os
    *  - Projectors
    *
    * For each component type, there exists a class definition that consists of
    * the following three fields:
    *  # name - A unique name for this instance of a component type
    *  # command - A class which is used to send commands to this component
    *  # status - A class which is used to return the status of this component
    *
    * Drivers that provide one or more of these components register the
    * corresponding class for that component by name with the HardwareInterface.
    * For a given component type, names must be unique.
    *
    * Controllers can retrieve a pointer to a component's class by name.  The
    * component is controlled using the command_ field of the component class,
    * and its status is given in the status_ field.
    */

    class HardwareInterface
    {
        private:
            ActuatorMap actuators;
            // PressureSensorMap pressuresensors;
            // AccelerometerMap accelerometers;
            // ForceTorqueMap ft_sensors;
            // DigitalOutMap digital_outs;
            // ProjectorMap projectors;
            // AnalogInMap analog_ins;
            // CustomHWMap custom_hws;
        public:
            Actuator* getActuator(const std::string &name) const
            {
                ActuatorMap::const_iterator iterator = actuators.find(name);

                return iterator != actuators.end() ? iterator->second : NULL;
            }
    };
}


#endif 