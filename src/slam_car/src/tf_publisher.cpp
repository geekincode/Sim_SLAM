#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("frame_publisher")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&FramePublisher::timer_callback, this));  // 20Hz更新频率
    
    // 从参数服务器获取变换参数，允许运行时调整
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("z", 0.5);
    this->declare_parameter("roll", 0.0);
    this->declare_parameter("pitch", 0.0);
    this->declare_parameter("yaw", 0.0);
    
    RCLCPP_INFO(this->get_logger(), "Frame Publisher node initialized");
  }

private:
  void timer_callback()
  {
    // 获取参数
    double x, y, z, roll, pitch, yaw;
    this->get_parameter("x", x);
    this->get_parameter("y", y);
    this->get_parameter("z", z);
    this->get_parameter("roll", roll);
    this->get_parameter("pitch", pitch);
    this->get_parameter("yaw", yaw);
    
    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_footprint";
    t.child_frame_id = "livox_mid360";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(t);
  }
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}