#include <iostream>
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cctype>
#include <memory>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl()
    : Node("keyboard_control"),
      wheel_speed_(15.0)
    {
        // 创建 Twist 命令发布者，发布到独立四轮速度控制话题
        // linear.x = FL, linear.y = FR, angular.x = BL, angular.y = BR
        cmd_wheel_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/model/slam_car/cmd_wheel_vel", 10);
        
        // 声明参数
        this->declare_parameter<double>("wheel_speed", 15.0);
        
        // 获取参数
        this->get_parameter("wheel_speed", wheel_speed_);
        
        printInstructions();
    }

    int kfd = 0;
    struct termios cooked, raw;

    char getKey()
    {
        fd_set fds;
        int ret;
        char c = 0;

        FD_ZERO(&fds);
        FD_SET(kfd, &fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10毫秒

        ret = select(kfd+1, &fds, NULL, NULL, &tv);

        if ((ret < 0) && (errno != EINTR))
        {
            perror("select error");
            exit(-1);
        }
        else if (ret == 0)
        {
            // 超时，没有按键
        }
        else
        {
            if (read(kfd, &c, 1) < 0)
            {
                perror("read error");
                exit(-1);
            }
        }

        return c;
    }

    void cleanup()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

    void publishWheelVel(double fl, double fr, double bl, double br)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        // linear.x = FL, linear.y = FR
        // angular.x = BL, angular.y = BR
        twist_msg.linear.x = fl;
        twist_msg.linear.y = fr;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = bl;
        twist_msg.angular.y = br;
        twist_msg.angular.z = 0.0;
        cmd_wheel_vel_pub_->publish(twist_msg);
    }

    void printInstructions()
    {
        std::cout << "\n========== Slam Car 键盘控制 ==========" << std::endl;
        std::cout << "运动控制:" << std::endl;
        std::cout << "  I         : 前进" << std::endl;
        std::cout << "  , (逗号)  : 后退" << std::endl;
        std::cout << "  J         : 左转" << std::endl;
        std::cout << "  L         : 右转" << std::endl;
        std::cout << "  U         : 左前移动" << std::endl;
        std::cout << "  O         : 右前移动" << std::endl;
        std::cout << "  N         : 左后移动" << std::endl;
        std::cout << "  M         : 右后移动" << std::endl;
        std::cout << "\n速度调节:" << std::endl;
        std::cout << "  W/S       : 增加/降低轮子速度" << std::endl;
        std::cout << "\n其他:" << std::endl;
        std::cout << "  空格/0    : 停止" << std::endl;
        std::cout << "  Q         : 退出程序" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "当前轮子速度: " << wheel_speed_ << " rad/s" << std::endl;
        std::cout << "提示：按住键可以持续控制，松开后自动停止" << std::endl;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_wheel_vel_pub_;
    double wheel_speed_;
};

void quit(int sig)
{
    (void)sig;
    std::cout << "\n关闭键盘控制节点..." << std::endl;
    exit(0);
}

int main(int argc, char** argv)
{
    signal(SIGINT, quit);
    
    rclcpp::init(argc, argv);
    auto keyboard_control = std::make_shared<KeyboardControl>();
    
    int kfd = 0;
    struct termios cooked, raw;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    char c;
    bool dirty = false;
    
    // 当前四个轮子的速度
    double fl_vel = 0.0;  // 左前轮
    double fr_vel = 0.0;  // 右前轮
    double bl_vel = 0.0;  // 左后轮
    double br_vel = 0.0;  // 右后轮
    
    std::cout << "\n键盘控制已启动！" << std::endl;
    
    // 主循环
    while (true) {
        c = keyboard_control->getKey();
        
        // 将字符转换为小写
        char lower_c = std::tolower(c);
        
        // 默认不发布，除非有按键
        dirty = false;
        
        switch (lower_c) {
            // ========== 前进/后退 ==========
            case 'i':  // 前进
                // 左轮正转，右轮反转
                fl_vel = bl_vel = keyboard_control->wheel_speed_;
                fr_vel = br_vel = -keyboard_control->wheel_speed_;
                dirty = true;
                std::cout << "前进: 速度=" << keyboard_control->wheel_speed_ << " rad/s" << std::endl;
                break;
                
            case ',':  // 后退
                // 左轮反转，右轮正转
                fl_vel = bl_vel = -keyboard_control->wheel_speed_;
                fr_vel = br_vel = keyboard_control->wheel_speed_;
                dirty = true;
                std::cout << "后退: 速度=" << keyboard_control->wheel_speed_ << " rad/s" << std::endl;
                break;
                
            // ========== 左转/右转 ==========
            case 'j':  // 左转（原地左转）
                // 左轮后退，右轮前进
                fl_vel = bl_vel = -keyboard_control->wheel_speed_;
                fr_vel = br_vel = -keyboard_control->wheel_speed_;
                dirty = true;
                std::cout << "左转" << std::endl;
                break;
                
            case 'l':  // 右转（原地右转）
                // 左轮前进，右轮后退
                fl_vel = bl_vel = keyboard_control->wheel_speed_;
                fr_vel = br_vel = keyboard_control->wheel_speed_;
                dirty = true;
                std::cout << "右转" << std::endl;
                break;
                
            // ========== 斜向移动 ==========
            case 'u':  // 左前移动
                fl_vel = bl_vel = keyboard_control->wheel_speed_;
                fr_vel = br_vel = 0.0;
                dirty = true;
                std::cout << "左前移动" << std::endl;
                break;
                
            case 'o':  // 右前移动
                fl_vel = bl_vel = 0.0;
                fr_vel = br_vel = -keyboard_control->wheel_speed_;
                dirty = true;
                std::cout << "右前移动" << std::endl;
                break;
                
            case 'n':  // 左后移动
                fl_vel = bl_vel = -keyboard_control->wheel_speed_;
                fr_vel = br_vel = 0.0;
                dirty = true;
                std::cout << "左后移动" << std::endl;
                break;
                
            case 'm':  // 右后移动
                fl_vel = bl_vel = 0.0;
                fr_vel = br_vel = keyboard_control->wheel_speed_;
                dirty = true;
                std::cout << "右后移动" << std::endl;
                break;
            
            // ========== 速度调节 ==========
            case 'w':  // 增加速度
                keyboard_control->wheel_speed_ += 1.0;
                keyboard_control->wheel_speed_ = std::min(keyboard_control->wheel_speed_, 30.0);
                std::cout << "速度增加到: " << keyboard_control->wheel_speed_ << " rad/s" << std::endl;
                break;
                
            case 's':  // 降低速度
                keyboard_control->wheel_speed_ -= 1.0;
                keyboard_control->wheel_speed_ = std::max(keyboard_control->wheel_speed_, 1.0);
                std::cout << "速度降低到: " << keyboard_control->wheel_speed_ << " rad/s" << std::endl;
                break;
            
            // ========== 停止 ==========
            case '0':  // 停止
            case ' ':  // 空格键停止
                fl_vel = fr_vel = bl_vel = br_vel = 0.0;
                dirty = true;
                std::cout << "停止" << std::endl;
                break;
            
            // ========== 退出 ==========
            case 'q':  // 退出
                std::cout << "\n关闭键盘控制节点..." << std::endl;
                // 发送停止命令
                keyboard_control->publishWheelVel(0.0, 0.0, 0.0, 0.0);
                keyboard_control->cleanup();
                rclcpp::shutdown();
                return 0;
                
            default:
                break;
        }
        
        if (dirty) {
            keyboard_control->publishWheelVel(fl_vel, fr_vel, bl_vel, br_vel);
        }
        
        rclcpp::spin_some(keyboard_control);
        usleep(10000); // 10毫秒延迟
    }
    
    keyboard_control->cleanup();
    rclcpp::shutdown();
    return 0;
}