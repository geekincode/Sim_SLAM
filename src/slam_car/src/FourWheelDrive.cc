/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "slam_car/FourWheelDrive.hh"

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>

#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <fstream>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

// Simple YAML parser for basic key-value pairs
namespace {
  class SimpleYamlParser {
    public:
      SimpleYamlParser(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
          ignwarn << "Failed to open YAML config file: " << filepath << std::endl;
          return;
        }
        std::string line;
        std::string current_section;
        while (std::getline(file, line)) {
          // Remove comments
          size_t comment_pos = line.find('#');
          if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
          }
          // Trim whitespace
          size_t start = line.find_first_not_of(" \t\r\n");
          if (start == std::string::npos) continue;
          size_t end = line.find_last_not_of(" \t\r\n");
          line = line.substr(start, end - start + 1);
          
          if (line.empty()) continue;
          
          // Check for section
          if (line.back() == ':') {
            current_section = line.substr(0, line.length() - 1);
            continue;
          }
          
          // Parse key-value pair
          size_t colon_pos = line.find(':');
          if (colon_pos != std::string::npos) {
            std::string key = line.substr(0, colon_pos);
            std::string value = line.substr(colon_pos + 1);
            // Trim key and value
            key = key.substr(key.find_first_not_of(" \t"));
            key = key.substr(0, key.find_last_not_of(" \t") + 1);
            value = value.substr(value.find_first_not_of(" \t"));
            value = value.substr(0, value.find_last_not_of(" \t") + 1);
            
            if (!current_section.empty()) {
              key = current_section + "/" + key;
            }
            data_[key] = value;
          }
        }
      }
      
      bool HasKey(const std::string& key) const {
        return data_.find(key) != data_.end();
      }
      
      std::string GetString(const std::string& key, const std::string& default_val = "") const {
        auto it = data_.find(key);
        if (it != data_.end()) {
          // Remove quotes if present
          std::string val = it->second;
          if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            val = val.substr(1, val.size() - 2);
          }
          return val;
        }
        return default_val;
      }
      
      double GetDouble(const std::string& key, double default_val = 0.0) const {
        auto it = data_.find(key);
        if (it != data_.end()) {
          try {
            return std::stod(it->second);
          } catch (...) {
            return default_val;
          }
        }
        return default_val;
      }
      
      std::vector<std::string> GetStringVector(const std::string& key) const {
        std::vector<std::string> result;
        auto it = data_.find(key);
        if (it != data_.end()) {
          std::string val = it->second;
          // Parse array format: ["item1", "item2"]
          size_t start = val.find('[');
          size_t end = val.find(']');
          if (start != std::string::npos && end != std::string::npos && end > start) {
            std::string content = val.substr(start + 1, end - start - 1);
            size_t pos = 0;
            while (pos < content.length()) {
              size_t comma = content.find(',', pos);
              std::string item;
              if (comma == std::string::npos) {
                item = content.substr(pos);
                pos = content.length();
              } else {
                item = content.substr(pos, comma - pos);
                pos = comma + 1;
              }
              // Trim and remove quotes
              size_t item_start = item.find_first_not_of(" \t\"'");
              if (item_start != std::string::npos) {
                size_t item_end = item.find_last_not_of(" \t\"'") + 1;
                result.push_back(item.substr(item_start, item_end - item_start));
              }
            }
          }
        }
        return result;
      }
      
    private:
      std::map<std::string, std::string> data_;
  };
}

using namespace gz;
using namespace gz::sim;
using namespace systems;

/// \brief Four wheel velocity command.
struct FourWheelCommands
{
  /// \brief Front left wheel velocity.
  double fl{0.0};
  /// \brief Front right wheel velocity.
  double fr{0.0};
  /// \brief Back left wheel velocity.
  double bl{0.0};
  /// \brief Back right wheel velocity.
  double br{0.0};
};

class ignition::gazebo::systems::FourWheelDrivePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Twist message with linear.x and angular.z
  public: void OnCmdVel(const msgs::Twist &_msg);

  /// \brief Callback for individual wheel velocity subscription
  /// \param[in] _msg Four wheel velocities
  public: void OnWheelVel(const msgs::Twist &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager.
  public: void UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update the wheel velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager.
  public: void UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Load parameters from YAML config file.
  /// \param[in] _filepath Path to YAML file.
  public: void LoadYamlConfig(const std::string &_filepath);

  /// \brief Load parameters from SDF.
  /// \param[in] _sdf SDF element.
  public: void LoadSdfParams(const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint entities
  public: Entity flJoint{kNullEntity};
  public: Entity frJoint{kNullEntity};
  public: Entity blJoint{kNullEntity};
  public: Entity brJoint{kNullEntity};

  /// \brief Joint names
  public: std::string flJointName{"lf_j"};
  public: std::string frJointName{"rf_j"};
  public: std::string blJointName{"lb_j"};
  public: std::string brJointName{"rb_j"};

  /// \brief Wheel speeds
  public: double flSpeed{0};
  public: double frSpeed{0};
  public: double blSpeed{0};
  public: double brSpeed{0};

  /// \brief Wheel radius
  public: double wheelRadius{0.03};

  /// \brief Wheel separation (width between left and right)
  public: double wheelSeparationWidth{0.15};

  /// \brief Wheel separation (length between front and back)
  public: double wheelSeparationLength{0.15};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief Update period for odometry.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief TF message publisher.
  public: transport::Node::Publisher tfPub;

  /// \brief Velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiter;

  /// \brief Previous wheel commands.
  public: FourWheelCommands lastCmd;

  /// \brief Target velocity from cmd_vel (Twist message).
  public: msgs::Twist targetVel;

  /// \brief Target individual wheel velocities.
  public: FourWheelCommands targetWheelVel;

  /// \brief Use individual wheel control or twist control.
  public: bool useIndividualControl{false};

  /// \brief Enable/disable state.
  public: bool enabled{true};

  /// \brief A mutex to protect commands.
  public: std::mutex mutex;

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;

  /// \brief YAML config file path.
  public: std::string configFile;

  /// \brief Topic names.
  public: std::string cmdVelTopic;
  public: std::string wheelVelTopic;
  public: std::string odomTopic;
  public: std::string tfTopic;
};

//////////////////////////////////////////////////
FourWheelDrive::FourWheelDrive()
  : dataPtr(std::make_unique<FourWheelDrivePrivate>())
{
}

//////////////////////////////////////////////////
void FourWheelDrivePrivate::LoadYamlConfig(const std::string &_filepath)
{
  SimpleYamlParser parser(_filepath);
  
  // Load joint names
  if (parser.HasKey("four_wheel_drive/front_left_joint"))
    this->flJointName = parser.GetString("four_wheel_drive/front_left_joint", this->flJointName);
  if (parser.HasKey("four_wheel_drive/front_right_joint"))
    this->frJointName = parser.GetString("four_wheel_drive/front_right_joint", this->frJointName);
  if (parser.HasKey("four_wheel_drive/back_left_joint"))
    this->blJointName = parser.GetString("four_wheel_drive/back_left_joint", this->blJointName);
  if (parser.HasKey("four_wheel_drive/back_right_joint"))
    this->brJointName = parser.GetString("four_wheel_drive/back_right_joint", this->brJointName);
  
  // Load wheel parameters
  this->wheelRadius = parser.GetDouble("four_wheel_drive/wheel_radius", this->wheelRadius);
  this->wheelSeparationWidth = parser.GetDouble("four_wheel_drive/wheel_separation_width", this->wheelSeparationWidth);
  this->wheelSeparationLength = parser.GetDouble("four_wheel_drive/wheel_separation_length", this->wheelSeparationLength);
  
  // Load velocity limits
  double maxVel = parser.GetDouble("four_wheel_drive/max_velocity", 10.0);
  double minVel = parser.GetDouble("four_wheel_drive/min_velocity", -10.0);
  
  this->limiter = std::make_unique<math::SpeedLimiter>();
  this->limiter->SetMaxVelocity(maxVel);
  this->limiter->SetMinVelocity(minVel);
  
  // Load topics
  this->cmdVelTopic = parser.GetString("four_wheel_drive/cmd_vel_topic", "");
  this->wheelVelTopic = parser.GetString("four_wheel_drive/wheel_vel_topic", "");
  this->odomTopic = parser.GetString("four_wheel_drive/odom_topic", "");
  
  ignmsg << "Loaded YAML config from: " << _filepath << std::endl;
}

//////////////////////////////////////////////////
void FourWheelDrivePrivate::LoadSdfParams(const std::shared_ptr<const sdf::Element> &_sdf)
{
  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  
  // Joint names
  if (_sdf->HasElement("front_left_joint"))
    this->flJointName = _sdf->Get<std::string>("front_left_joint");
  if (_sdf->HasElement("front_right_joint"))
    this->frJointName = _sdf->Get<std::string>("front_right_joint");
  if (_sdf->HasElement("back_left_joint"))
    this->blJointName = _sdf->Get<std::string>("back_left_joint");
  if (_sdf->HasElement("back_right_joint"))
    this->brJointName = _sdf->Get<std::string>("back_right_joint");
  
  // Wheel parameters
  this->wheelRadius = _sdf->Get<double>("wheel_radius", this->wheelRadius).first;
  this->wheelSeparationWidth = _sdf->Get<double>("wheel_separation_width", this->wheelSeparationWidth).first;
  this->wheelSeparationLength = _sdf->Get<double>("wheel_separation_length", this->wheelSeparationLength).first;
  
  // Velocity limits
  double maxVel = _sdf->Get<double>("max_velocity", 10.0).first;
  double minVel = _sdf->Get<double>("min_velocity", -10.0).first;
  
  this->limiter = std::make_unique<math::SpeedLimiter>();
  this->limiter->SetMaxVelocity(maxVel);
  this->limiter->SetMinVelocity(minVel);
  
  // Config file path
  if (_sdf->HasElement("config_file"))
    this->configFile = _sdf->Get<std::string>("config_file");
  
  // Topics
  if (_sdf->HasElement("topic"))
    this->cmdVelTopic = _sdf->Get<std::string>("topic");
  if (_sdf->HasElement("wheel_vel_topic"))
    this->wheelVelTopic = _sdf->Get<std::string>("wheel_vel_topic");
  if (_sdf->HasElement("odom_topic"))
    this->odomTopic = _sdf->Get<std::string>("odom_topic");
  if (_sdf->HasElement("tf_topic"))
    this->tfTopic = _sdf->Get<std::string>("tf_topic");
  
  // Frame IDs
  if (_sdf->HasElement("frame_id"))
    this->sdfFrameId = _sdf->Get<std::string>("frame_id");
  if (_sdf->HasElement("child_frame_id"))
    this->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");
}

//////////////////////////////////////////////////
void FourWheelDrive::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "FourWheelDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Load SDF parameters first
  this->dataPtr->LoadSdfParams(_sdf);
  
  // If config file is specified, load YAML parameters (overrides SDF)
  if (!this->dataPtr->configFile.empty())
  {
    this->dataPtr->LoadYamlConfig(this->dataPtr->configFile);
  }

  // Setup velocity limiter if not already set
  if (!this->dataPtr->limiter)
  {
    this->dataPtr->limiter = std::make_unique<math::SpeedLimiter>();
    this->dataPtr->limiter->SetMaxVelocity(10.0);
    this->dataPtr->limiter->SetMinVelocity(-10.0);
  }

  // Setup odometry publish period
  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Subscribe to twist commands
  std::vector<std::string> topics;
  if (!this->dataPtr->cmdVelTopic.empty())
  {
    topics.push_back(this->dataPtr->cmdVelTopic);
  }
  topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel_4wd");
  auto topic = validTopic(topics);

  this->dataPtr->node.Subscribe(topic, &FourWheelDrivePrivate::OnCmdVel,
      this->dataPtr.get());

  // Subscribe to individual wheel velocity commands
  std::vector<std::string> wheelTopics;
  if (!this->dataPtr->wheelVelTopic.empty())
  {
    wheelTopics.push_back(this->dataPtr->wheelVelTopic);
  }
  wheelTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_wheel_vel");
  auto wheelTopic = validTopic(wheelTopics);

  this->dataPtr->node.Subscribe(wheelTopic, &FourWheelDrivePrivate::OnWheelVel,
      this->dataPtr.get());

  // Setup odometry publisher
  std::vector<std::string> odomTopics;
  if (!this->dataPtr->odomTopic.empty())
  {
    odomTopics.push_back(this->dataPtr->odomTopic);
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/odometry_4wd");
  auto odomTopic = validTopic(odomTopics);

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  // Setup TF publisher
  std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/tf_4wd"};
  if (!this->dataPtr->tfTopic.empty())
    tfTopic = this->dataPtr->tfTopic;
  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopic);

  ignmsg << "FourWheelDrive subscribing to twist messages on [" << topic << "]"
         << " and wheel velocities on [" << wheelTopic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void FourWheelDrive::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("FourWheelDrive::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  
  if (this->dataPtr->flJoint == kNullEntity)
  {
    this->dataPtr->flJoint = this->dataPtr->model.JointByName(_ecm, this->dataPtr->flJointName);
    if (this->dataPtr->flJoint == kNullEntity && 
        warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find front left joint [" << this->dataPtr->flJointName 
              << "] for model [" << modelName << "]" << std::endl;
    }
  }
  
  if (this->dataPtr->frJoint == kNullEntity)
  {
    this->dataPtr->frJoint = this->dataPtr->model.JointByName(_ecm, this->dataPtr->frJointName);
    if (this->dataPtr->frJoint == kNullEntity && 
        warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find front right joint [" << this->dataPtr->frJointName 
              << "] for model [" << modelName << "]" << std::endl;
    }
  }
  
  if (this->dataPtr->blJoint == kNullEntity)
  {
    this->dataPtr->blJoint = this->dataPtr->model.JointByName(_ecm, this->dataPtr->blJointName);
    if (this->dataPtr->blJoint == kNullEntity && 
        warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find back left joint [" << this->dataPtr->blJointName 
              << "] for model [" << modelName << "]" << std::endl;
    }
  }
  
  if (this->dataPtr->brJoint == kNullEntity)
  {
    this->dataPtr->brJoint = this->dataPtr->model.JointByName(_ecm, this->dataPtr->brJointName);
    if (this->dataPtr->brJoint == kNullEntity && 
        warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find back right joint [" << this->dataPtr->brJointName 
              << "] for model [" << modelName << "]" << std::endl;
    }
  }
  
  // Check if all joints are found
  bool allFound = (this->dataPtr->flJoint != kNullEntity &&
                   this->dataPtr->frJoint != kNullEntity &&
                   this->dataPtr->blJoint != kNullEntity &&
                   this->dataPtr->brJoint != kNullEntity);
  
  if (!allFound)
  {
    warnedModels.insert(modelName);
    return;
  }
  
  if (warnedModels.find(modelName) != warnedModels.end())
  {
    ignmsg << "Found all joints for model [" << modelName
           << "], FourWheelDrive plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Update wheel velocities
  auto updateJointVel = [&](Entity joint, double speed)
  {
    if (!_ecm.HasEntity(joint))
      return;
    
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    if (vel == nullptr)
    {
      _ecm.CreateComponent(joint, components::JointVelocityCmd({speed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({speed});
    }
  };

  updateJointVel(this->dataPtr->flJoint, this->dataPtr->flSpeed);
  updateJointVel(this->dataPtr->frJoint, this->dataPtr->frSpeed);
  updateJointVel(this->dataPtr->blJoint, this->dataPtr->blSpeed);
  updateJointVel(this->dataPtr->brJoint, this->dataPtr->brSpeed);

  // Create joint position components if they don't exist
  auto flPos = _ecm.Component<components::JointPosition>(this->dataPtr->flJoint);
  if (!flPos && _ecm.HasEntity(this->dataPtr->flJoint))
  {
    _ecm.CreateComponent(this->dataPtr->flJoint, components::JointPosition());
  }

  auto frPos = _ecm.Component<components::JointPosition>(this->dataPtr->frJoint);
  if (!frPos && _ecm.HasEntity(this->dataPtr->frJoint))
  {
    _ecm.CreateComponent(this->dataPtr->frJoint, components::JointPosition());
  }
}

//////////////////////////////////////////////////
void FourWheelDrive::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("FourWheelDrive::PostUpdate");
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void FourWheelDrivePrivate::UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("FourWheelDrive::UpdateOdometry");
  
  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Get joint positions
  auto flPos = _ecm.Component<components::JointPosition>(this->flJoint);
  auto frPos = _ecm.Component<components::JointPosition>(this->frJoint);

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(0);
  msg.mutable_pose()->mutable_position()->set_y(0);
  msg.mutable_twist()->mutable_linear()->set_x(0);
  msg.mutable_twist()->mutable_angular()->set_z(0);

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (this->sdfChildFrameId.empty())
  {
    if (linkName)
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Publish the messages
  this->odomPub.Publish(msg);
}

//////////////////////////////////////////////////
void FourWheelDrivePrivate::UpdateVelocity(const UpdateInfo &/*_info*/,
    const EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("FourWheelDrive::UpdateVelocity");

  double linVel = 0;
  double angVel = 0;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
  }

  if (this->useIndividualControl)
  {
    // Use individual wheel velocities
    std::lock_guard<std::mutex> lock(this->mutex);
    this->flSpeed = this->targetWheelVel.fl;
    this->frSpeed = this->targetWheelVel.fr;
    this->blSpeed = this->targetWheelVel.bl;
    this->brSpeed = this->targetWheelVel.br;
  }
  else
  {
    // Convert twist to wheel velocities (differential drive kinematics)
    // For a 4-wheel robot, we use the same formula as diff drive
    // but apply to all wheels
    double rightSpeed = (linVel + angVel * this->wheelSeparationWidth / 2.0) / this->wheelRadius;
    double leftSpeed = (linVel - angVel * this->wheelSeparationWidth / 2.0) / this->wheelRadius;
    
    // Limit velocities
    if (this->limiter)
    {
      this->limiter->LimitVelocity(rightSpeed);
      this->limiter->LimitVelocity(leftSpeed);
    }
    
    this->flSpeed = leftSpeed;
    this->blSpeed = leftSpeed;
    this->frSpeed = rightSpeed;
    this->brSpeed = rightSpeed;
  }
}

//////////////////////////////////////////////////
void FourWheelDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->enabled)
  {
    this->targetVel = _msg;
    this->useIndividualControl = false;
  }
}

//////////////////////////////////////////////////
void FourWheelDrivePrivate::OnWheelVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->enabled)
  {
    // Use Twist message to carry 4 wheel velocities
    // linear.x = front left, linear.y = front right
    // angular.x = back left, angular.y = back right
    this->targetWheelVel.fl = _msg.linear().x();
    this->targetWheelVel.fr = _msg.linear().y();
    this->targetWheelVel.bl = _msg.angular().x();
    this->targetWheelVel.br = _msg.angular().y();
    this->useIndividualControl = true;
  }
}

IGNITION_ADD_PLUGIN(FourWheelDrive,
                    System,
                    FourWheelDrive::ISystemConfigure,
                    FourWheelDrive::ISystemPreUpdate,
                    FourWheelDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(FourWheelDrive, "gz::sim::systems::FourWheelDrive")

// TODO(CH3): Deprecated, remove on version 8
IGNITION_ADD_PLUGIN_ALIAS(FourWheelDrive, "ignition::gazebo::systems::FourWheelDrive")