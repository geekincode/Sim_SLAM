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
#ifndef GZ_GAZEBO_SYSTEMS_FOURWHEELDRIVE_HH_
#define GZ_GAZEBO_SYSTEMS_FOURWHEELDRIVE_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/sim/System.hh>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class FourWheelDrivePrivate;

  /// \brief Four wheel independent drive controller.
  /// Each wheel can be controlled independently via velocity commands.
  /// Parameters can be loaded from YAML configuration file.
  ///
  /// # System Parameters
  ///
  /// `<front_left_joint>`: Name of front left wheel joint.
  /// `<front_right_joint>`: Name of front right wheel joint.
  /// `<back_left_joint>`: Name of back left wheel joint.
  /// `<back_right_joint>`: Name of back right wheel joint.
  ///
  /// `<wheel_radius>`: Wheel radius in meters. Default: 0.03m.
  /// `<wheel_separation_width>`: Distance between left and right wheels. Default: 0.15m.
  /// `<wheel_separation_length>`: Distance between front and back wheels. Default: 0.15m.
  ///
  /// `<max_velocity>`: Maximum wheel velocity in rad/s. Default: 10.0.
  /// `<min_velocity>`: Minimum wheel velocity in rad/s. Default: -10.0.
  ///
  /// `<topic>`: Command topic. Default: `/model/{model_name}/cmd_vel_4wd`.
  /// `<odom_topic>`: Odometry topic. Default: `/model/{model_name}/odometry_4wd`.
  /// `<odom_publish_frequency>`: Odometry publish frequency in Hz. Default: 50.0.
  ///
  /// `<config_file>`: Path to YAML config file for parameters.
  class FourWheelDrive
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: FourWheelDrive();

    /// \brief Destructor
    public: ~FourWheelDrive() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<FourWheelDrivePrivate> dataPtr;
  };
}
}
}
}

#endif