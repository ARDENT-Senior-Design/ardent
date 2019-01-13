#include "../include/ardent/robot.h"

using namespace ardent_model;
using namespace ardent_hardware_interface;

Robot::Robot(std::vector<std::string> legs, HardwareInterface *hw) : hw_(hw)
{
    num_legs_ = legs.size();
    
    for(int i=0;i<legs.size();i++){
        legs_.push_back(LegKinematics(legs[i], body_.GetRadius()));
    }
    //initialize the legs based on the body offset
}

ros::Time RobotState::getTime()
{
    return current_time_;
}

void Robot::publishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos)
{
    int leg_map = getMappedLeg(leg_id);
    sensor_msgs::JointState joint_state= legs_[leg_map].getJointState(ee_pos);
    legs_[leg_map].publishJointState(joint_state);
}

std::string Robot::getMappedLeg(int leg_num)
{
    static const std::map<int,std::string> leg_map{
        {0, "rf"},
        {1,"rm"},
        {2, "rr"},
        {3, "lf"},
        {4, "lm"},
        {5, "lr"}
    };
    return leg_map.at(leg_num);
}
int Robot::getMappedLeg(std::string leg_id)
{
    static const std::map<std::string, int> leg_map{
        {"rf", 0},
        {"rm", 1},
        {"rr", 2},
        {"lf", 3},
        {"lm", 4},
        {"lr", 5}
    };return leg_map.at(leg_id);

}
bool Robot::checkStability()
{
    std::vector<float> contact_legs;
    for(int i=0;i<num_legs_;i++){
        //if(GetMappedLeg(i))
    }
}

bool Robot::initXml(TiXmlElement *root)
{
  // check if current time is valid
  if (!hw_){
    ROS_ERROR("Mechanism Model received an invalid hardware interface");
    return false;
  }

  // Parses the xml into a robot model
  if (!robot_model_.initXml(root)){
    ROS_ERROR("Mechanism Model failed to parse the URDF xml into a robot model");
    return false;
  }

  // Creates the plugin loader for transmissions.
  transmission_loader_.reset(new pluginlib::ClassLoader<ardent_model::Transmission>(
                               "pr2_mechanism_model", "pr2_mechanism_model::Transmission"));

  // Constructs the transmissions by parsing custom xml.
  TiXmlElement *xit = NULL;
  for (xit = root->FirstChildElement("transmission"); xit;
       xit = xit->NextSiblingElement("transmission"))
  {
    std::string type(xit->Attribute("type"));
    boost::shared_ptr<Transmission> t;
    try {
      // Backwards compatibility for using non-namespaced controller types
      if (!transmission_loader_->isClassAvailable(type))
      {
        std::vector<std::string> classes = transmission_loader_->getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i)
        {
          if(type == transmission_loader_->getName(classes[i]))
          {
            ROS_WARN("The deprecated transmission type %s was not found.  Using the namespaced version %s instead.  "
                     "Please update your urdf file to use the namespaced version.",
                     type.c_str(), classes[i].c_str());
            type = classes[i];
            break;
          }
        }
      }
      t = transmission_loader_->createInstance(type);
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
    }

    if (!t)
      ROS_ERROR("Unknown transmission type: %s", type.c_str());
    else if (!t->initXml(xit, this)){
      ROS_ERROR("Failed to initialize transmission");
    }
    else // Success!
      transmissions_.push_back(t);
  }

  return true;
}

ros::Time Robot::getTime()
{
  return hw_->current_time_;
}

template <class T>
int findIndexByName(const std::vector<boost::shared_ptr<T> >& v, 
      const std::string &name)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i]->name_ == name)
      return i;
  }
  return -1;
}

int Robot::getTransmissionIndex(const std::string &name) const
{
  return findIndexByName(transmissions_, name);
}

Actuator* Robot::getActuator(const std::string &name) const
{
  return hw_->getActuator(name);
}

boost::shared_ptr<Transmission> Robot::getTransmission(const std::string &name) const
{
  int i = getTransmissionIndex(name);
  return i >= 0 ? transmissions_[i] : boost::shared_ptr<Transmission>();
}





RobotState::RobotState(Robot *model)
  : model_(model)
{
  assert(model_);

  transmissions_in_.resize(model->transmissions_.size());
  transmissions_out_.resize(model->transmissions_.size());

  // Creates a joint state for each transmission
  unsigned int js_size = 0;
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
     boost::shared_ptr<Transmission> t = model_->transmissions_[i];
    for (unsigned int j = 0; j < t->actuator_names_.size(); ++j)
    {
      Actuator *act = model_->getActuator(t->actuator_names_[j]);
      assert(act != NULL);
      transmissions_in_[i].push_back(act);
    }
    js_size += t->joint_names_.size();
  }

  // Wires up the transmissions to the joint state
  joint_states_.resize(js_size);
  unsigned int js_id = 0;
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
     boost::shared_ptr<Transmission> t = model_->transmissions_[i];
    for (unsigned int j = 0; j < t->joint_names_.size(); ++j)
    {
      joint_states_[js_id].joint_ = model_->robot_model_.getJoint(t->joint_names_[j]);
      joint_states_map_[t->joint_names_[j]] = &(joint_states_[js_id]);
      transmissions_out_[i].push_back(&(joint_states_[js_id]));
      js_id++;
    }
  }

  // warnings
  if (model_->transmissions_.empty())
    ROS_WARN("No transmissions were specified in the robot description.");
  if (js_size == 0)
    ROS_WARN("None of the joints in the robot desription matches up to a motor. The robot is uncontrollable.");
}


JointState *RobotState::getJointState(const std::string &name)
{
  std::map<std::string, JointState*>::iterator it = joint_states_map_.find(name);
  if (it == joint_states_map_.end())
    return NULL;
  else
    return it->second;
}

const JointState *RobotState::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_states_map_.find(name);
  if (it == joint_states_map_.end())
    return NULL;
  else
    return it->second;
}

void RobotState::propagateActuatorPositionToJointPosition()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagatePosition(transmissions_in_[i],
                                                 transmissions_out_[i]);
  }

  for (unsigned int i = 0; i < joint_states_.size(); i++)
  {
    joint_states_[i].joint_statistics_.update(&(joint_states_[i]));
  }
}

void RobotState::propagateJointEffortToActuatorEffort()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagateEffort(transmissions_out_[i],
                                               transmissions_in_[i]);
  }
}

bool RobotState::isHalted()
{
  for (unsigned int t = 0; t < transmissions_in_.size(); ++t){
    for (unsigned int a = 0; a < transmissions_in_[t].size(); a++){
      if (transmissions_in_[t][a]->state_.halted_)
        return true;
    }
  }

  return false;
}

void RobotState::enforceSafety()
{
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
  {
    joint_states_[i].enforceLimits();
  }
}

void RobotState::zeroCommands()
{
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
    joint_states_[i].commanded_effort_ = 0;
}

void RobotState::propagateJointPositionToActuatorPosition()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagatePositionBackwards(transmissions_out_[i],
                                                          transmissions_in_[i]);
  }
}

void RobotState::propagateActuatorEffortToJointEffort()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagateEffortBackwards(transmissions_in_[i],
                                                        transmissions_out_[i]);
  }
}


