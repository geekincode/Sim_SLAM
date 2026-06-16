#ifndef GAZEBO_WHEEL_CONTROLLER_H
#define GAZEBO_WHEEL_CONTROLLER_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <string>
#include <memory>

namespace slam_car
{

class GazeboWheelController : public gazebo::ModelPlugin
{
public:
    GazeboWheelController();
    ~GazeboWheelController() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
    void OnUpdate(const gazebo::common::UpdateInfo & info);

private:
    void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void wheelVelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    void publishJointStates();
    void updateWheelVelocities();
    
    // Gazebo相关
    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr lf_joint_;  // 左前轮
    gazebo::physics::JointPtr lb_joint_;  // 左后轮
    gazebo::physics::JointPtr rf_joint_;  // 右前轮
    gazebo::physics::JointPtr rb_joint_;  // 右后轮
    
    gazebo::event::ConnectionPtr update_connection_;
    
    // ROS2相关
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 轮子参数
    double wheel_radius_;
    double wheel_separation_width_;
    double wheel_separation_length_;
    double max_wheel_torque_;
    double max_wheel_accel_;
    
    // 目标速度
    double lf_target_vel_;
    double lb_target_vel_;
    double rf_target_vel_;
    double rb_target_vel_;
    
    // 当前速度
    double lf_current_vel_;
    double lb_current_vel_;
    double rf_current_vel_;
    double rb_current_vel_;
    
    // 控制模式
    enum ControlMode
    {
        INDIVIDUAL_WHEEL,  // 独立轮子控制
        JOINT_STATE,       // JointState控制
        CMD_VEL            // 速度指令控制
    };
    ControlMode control_mode_;
    
    // 发布频率
    double publish_rate_;
    
    // 关节名称
    std::vector<std::string> wheel_joint_names_;
};

}

#endif // GAZEBO_WHEEL_CONTROLLER_H