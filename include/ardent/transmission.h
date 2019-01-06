#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include <tinyxml.h>
#include "joint.h"
#include "hardware_interface.h"
namespace ardent_model
{
    class Robot;

    class Transmission
    {
        public:
            Transmission(){}

            virtual ~Transmission(){}

            // Fill out joint position and velocities using encoder data
            virtual void propagatePositions(std::vector<ardent_hardware_interface::Actuator*>&,
                                                    std::vector<ardent_model::JointState*>&) =0;

            // Uses the joint position to fill out the actuator encoder
            virtual void propagatePositionBackwards(std::vector<ardent_hardware_interface::Actuator*>&,
                                                    std::vector<ardent_model::JointState*>&) =0;
            
            // Uses the commanded joint efforts to fill out commanded motor currents
            virtual void propagateEffort(std::vector<ardent_hardware_interface::Actuator*>&,
                                            std::vector<ardent_model::JointState*>&) =0;

            // Uses the actuators commanded effort to fill out the torque on the joint
            virtual void propagateEffortBackwards(std::vector<ardent_hardware_interface::Actuator*>&,
                                                  std::vector<ardent_model::JointState*>&) =0;
            
            std::string transmission_name;
            std::vector<std::string> actuator_names;
            std::vector<std::string> joint_names;

            // initialize transmission from xml data
            virtual bool initXml(TiXmlElement *config) { abort(); }
    };
}


#endif