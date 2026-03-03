#include "slam_car/custom_slam_controller.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace slam_car
{
    CustomSlamController::CustomSlamController(const rclcpp::NodeOptions & options)
    : Node("custom_slam_controller", options),
      x_pos_(0.0), y_pos_(0.0), theta_pos_(0.0),
      x_vel_(0.0), y_vel_(0.0), theta_vel_(0.0),
      last_update_time_(this->now())
    {
        // 声明参数
        this->declare_parameter<std::string>("robot_base_frame_id", "base_footprint");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<double>("cmd_vel_timeout", 0.5);
        this->declare_parameter<bool>("enable_odom_tf", true);
        this->declare_parameter<double>("wheel_radius", 0.03);
        this->declare_parameter<double>("wheel_separation_width", 0.15);
        this->declare_parameter<double>("wheel_separation_length", 0.10); // 车身长度的一半
        this->declare_parameter<std::vector<std::string>>("wheel_joint_names", 
            std::vector<std::string>({"lf_j", "lb_j", "rf_j", "rb_j"}));
        
        // 获取参数
        this->get_parameter("robot_base_frame_id", robot_base_frame_id_);
        this->get_parameter("odom_frame_id", odom_frame_id_);
        this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
        this->get_parameter("enable_odom_tf", enable_odom_tf_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("wheel_separation_width", wheel_separation_width_);
        this->get_parameter("wheel_separation_length", wheel_separation_length_);
        this->get_parameter("wheel_joint_names", wheel_joint_names_);
        
        // 初始化关节位置和速度向量
        size_t num_wheels = wheel_joint_names_.size();
        wheel_positions_.resize(num_wheels, 0.0);
        wheel_velocities_.resize(num_wheels, 0.0);
        
        // 创建发布者和订阅者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&CustomSlamController::cmdVelCallback, this, std::placeholders::_1));
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&CustomSlamController::jointStateCallback, this, std::placeholders::_1));
        joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
        
        // 初始化TF广播器
        if (enable_odom_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        
        // 创建定时器用于发布里程计和关节控制
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz
            std::bind(&CustomSlamController::publishOdom, this));
    }

    void CustomSlamController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 将线速度和角速度转换为四个轮子的速度
        x_vel_ = msg->linear.x;
        y_vel_ = msg->linear.y;
        theta_vel_ = msg->angular.z;
        
        // 计算每个轮子的速度和方向（对于全向移动机器人）
        // 这里假设四个轮子都可以独立控制速度和转向
        double v_left_front = sqrt(2.0) * (x_vel_ - y_vel_ - theta_vel_ * (wheel_separation_width_/2.0 + wheel_separation_length_/2.0));
        double v_right_front = sqrt(2.0) * (x_vel_ + y_vel_ + theta_vel_ * (wheel_separation_width_/2.0 + wheel_separation_length_/2.0));
        double v_left_back = sqrt(2.0) * (x_vel_ + y_vel_ - theta_vel_ * (wheel_separation_width_/2.0 + wheel_separation_length_/2.0));
        double v_right_back = sqrt(2.0) * (x_vel_ - y_vel_ + theta_vel_ * (wheel_separation_width_/2.0 + wheel_separation_length_/2.0));
        
        // 创建关节命令消息
        sensor_msgs::msg::JointState joint_cmd_msg;
        joint_cmd_msg.header.stamp = this->now();
        joint_cmd_msg.name = wheel_joint_names_;
        joint_cmd_msg.velocity = {v_left_front, v_left_back, v_right_front, v_right_back};
        
        // 发布关节命令
        joint_cmd_pub_->publish(joint_cmd_msg);
    }

    void CustomSlamController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 更新关节位置和速度
        for(size_t i = 0; i < wheel_joint_names_.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), wheel_joint_names_[i]);
            if(it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                
                if(index < msg->position.size()) {
                    wheel_positions_[i] = msg->position[index];
                }
                
                if(index < msg->velocity.size()) {
                    wheel_velocities_[i] = msg->velocity[index];
                }
            }
        }
        
        // 使用关节状态更新里程计
        auto current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;
        
        if(dt > 0) {
            // 简单的里程计算法 - 根据轮子速度积分得到位置
            double dx = x_vel_ * dt;
            double dy = y_vel_ * dt;
            double dtheta = theta_vel_ * dt;
            
            // 更新全局位置
            x_pos_ += cos(theta_pos_) * dx - sin(theta_pos_) * dy;
            y_pos_ += sin(theta_pos_) * dx + cos(theta_pos_) * dy;
            theta_pos_ += dtheta;
        }
    }

    void CustomSlamController::publishOdom()
    {
        // 创建里程计消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = robot_base_frame_id_;
        
        // 设置位置
        odom_msg.pose.pose.position.x = x_pos_;
        odom_msg.pose.pose.position.y = y_pos_;
        odom_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta_pos_);
        odom_msg.pose.pose.orientation = tf2::toMsg(quat);
        
        // 设置速度
        odom_msg.twist.twist.linear.x = x_vel_;
        odom_msg.twist.twist.linear.y = y_vel_;
        odom_msg.twist.twist.angular.z = theta_vel_;
        
        // 设置协方差矩阵
        for(int i = 0; i < 6; i++) {
            odom_msg.pose.covariance[i*6+i] = 0.01;
            odom_msg.twist.covariance[i*6+i] = 0.01;
        }
        
        // 发布里程计
        odom_pub_->publish(odom_msg);
        
        // 如果启用，则发布TF变换
        if (enable_odom_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();
            transform.header.frame_id = odom_frame_id_;
            transform.child_frame_id = robot_base_frame_id_;
            
            transform.transform.translation.x = x_pos_;
            transform.transform.translation.y = y_pos_;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation = tf2::toMsg(quat);
            
            tf_broadcaster_->sendTransform(transform);
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(slam_car::CustomSlamController)