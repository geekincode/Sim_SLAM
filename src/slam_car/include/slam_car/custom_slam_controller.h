#ifndef CUSTOM_SLAM_CONTROLLER_H
#define CUSTOM_SLAM_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <vector>

namespace slam_car
{
    class CustomSlamController : public rclcpp::Node
    {
    public:
        explicit CustomSlamController(const rclcpp::NodeOptions & options);
        
    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void publishOdom();
        void publishJointControl();
        
        // 发布者和订阅者
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        
        // 控制器参数
        std::string robot_base_frame_id_;
        std::string odom_frame_id_;
        double cmd_vel_timeout_;
        bool enable_odom_tf_;
        double wheel_radius_;
        double wheel_separation_width_;
        double wheel_separation_length_;
        
        // 里程计相关变量
        double x_pos_, y_pos_, theta_pos_;
        double x_vel_, y_vel_, theta_vel_;
        std::vector<std::string> wheel_joint_names_;
        std::vector<double> wheel_positions_;
        std::vector<double> wheel_velocities_;
        rclcpp::Time last_update_time_;
        
        // TF变换广播器
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        
        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif // CUSTOM_SLAM_CONTROLLER_H