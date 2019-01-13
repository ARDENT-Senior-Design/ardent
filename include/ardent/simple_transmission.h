#ifndef SIMPLE_TRANSMISSION_H_
#define SIMPLE_TRANSMISSION_H_

#include <tinyxml.h>
#include "transmission.h"
#include "joint.h"
#include "hardware_interface.h"
#include <math.h>
#include <pluginlib/class_list_macros.h>
//#include "ardent_model/joint_calibration_simulator.h"

namespace ardent_model {

class SimpleTransmission : public Transmission
{
  public:
    SimpleTransmission() {use_simulated_actuated_joint_=true;}
    ~SimpleTransmission() {}

    bool initXml(TiXmlElement *config, Robot *robot);
    bool initXml(TiXmlElement *config);

    double mechanical_reduction_;

    void propagatePosition(std::vector<ardent_hardware_interface::Actuator*>&,
                          std::vector<ardent_model::JointState*>&);
    void propagatePositionBackwards(std::vector<ardent_model::JointState*>&,
                                    std::vector<ardent_hardware_interface::Actuator*>&);
    void propagateEffort(std::vector<ardent_model::JointState*>&,
                        std::vector<ardent_hardware_interface::Actuator*>&);
    void propagateEffortBackwards(std::vector<ardent_hardware_interface::Actuator*>&,
                                  std::vector<ardent_model::JointState*>&);

  private:
    // if a actuated_joint is specified, apply torque based on simulated_reduction_
    double simulated_reduction_;
    bool use_simulated_actuated_joint_;

    int simulated_actuator_timestamp_initialized_;
    ros::Time simulated_actuator_start_time_;

    //JointCalibrationSimulator joint_calibration_simulator_;
  };
}

#endif
