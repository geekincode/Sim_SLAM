#include <iostream>
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cctype>  // 用于tolower函数
#include <memory>  // 用于std::make_shared

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6b
#define KEYCODE_J 0x6a
#define KEYCODE_L 0x6c
#define KEYCODE_COMMA 0x2c
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_N 0x6E
#define KEYCODE_M 0x6D

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // 获取参数或设置默认值
        this->declare_parameter<double>("linear_speed", 0.5);
        this->declare_parameter<double>("angular_speed", 0.5);
        
        this->get_parameter("linear_speed", linear_speed_);
        this->get_parameter("angular_speed", angular_speed_);
        
        std::cout << "请按以下键控制机器人运动：" << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << "   I    : 前进" << std::endl;
        std::cout << "   ,    : 后退" << std::endl;
        std::cout << "   J/K  : 左转/右转" << std::endl;
        std::cout << "   U/O  : 左前/右前" << std::endl;
        std::cout << "   N/M  : 左后/右后" << std::endl;
        std::cout << "   Q    : 退出程序" << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << "提示：按住键可以让机器人持续运动" << std::endl;
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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    double linear_speed_, angular_speed_;
};

void quit(int sig)
{
    (void)sig;
    std::cout << "关闭键盘控制节点..." << std::endl;
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
    geometry_msgs::msg::Twist twist;
    
    std::cout << "请按以下键控制机器人运动：" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "   I    : 前进" << std::endl;
    std::cout << "   ,    : 后退" << std::endl;
    std::cout << "   J/K  : 左转/右转" << std::endl;
    std::cout << "   U/O  : 左前/右前" << std::endl;
    std::cout << "   N/M  : 左后/右后" << std::endl;
    std::cout << "   Q    : 退出程序" << std::endl;
    std::cout << "---------------------------" << std::endl;
    
    // 主循环
    while (true) {
        c = keyboard_control->getKey();

        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;
        
        // 将字符转换为小写以统一处理大写和小写字母
        char lower_c = std::tolower(c);
        
        switch (lower_c) {
            case 'i':
                twist.linear.x = keyboard_control->linear_speed_;
                dirty = true;
                break;
            case ',':  // Backward
                twist.linear.x = -keyboard_control->linear_speed_;
                dirty = true;
                break;
            case 'j':  // Turn left
                twist.angular.z = keyboard_control->angular_speed_;
                dirty = true;
                break;
            case 'l':  // Turn right
                twist.angular.z = -keyboard_control->angular_speed_;
                dirty = true;
                break;
            case 'u':  // Forward-left
                twist.linear.x = keyboard_control->linear_speed_;
                twist.angular.z = keyboard_control->angular_speed_;
                dirty = true;
                break;
            case 'o':  // Forward-right
                twist.linear.x = keyboard_control->linear_speed_;
                twist.angular.z = -keyboard_control->angular_speed_;
                dirty = true;
                break;
            case 'n':  // Backward-left
                twist.linear.x = -keyboard_control->linear_speed_;
                twist.angular.z = keyboard_control->angular_speed_;
                dirty = true;
                break;
            case 'm':  // Backward-right
                twist.linear.x = -keyboard_control->linear_speed_;
                twist.angular.z = -keyboard_control->angular_speed_;
                dirty = true;
                break;
            case 'q':  // Quit
                std::cout << "关闭键盘控制节点..." << std::endl;
                keyboard_control->cleanup();
                rclcpp::shutdown();
                return 0;
            default:
                break;
        }
        
        if (dirty) {
            keyboard_control->cmd_pub_->publish(twist);
            dirty = false;
        }
        
        rclcpp::spin_some(keyboard_control);
        usleep(10000); // 10毫秒延迟
    }
    
    keyboard_control->cleanup();
    rclcpp::shutdown();
    return 0;
}