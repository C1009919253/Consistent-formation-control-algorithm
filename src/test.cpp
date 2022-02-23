#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termio.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

int scanKeyboard()
{
int in;
struct termios new_settings;
struct termios stored_settings;
tcgetattr(0,&stored_settings);
new_settings = stored_settings;
new_settings.c_lflag &= (~ICANON);
new_settings.c_cc[VTIME] = 0;
tcgetattr(0,&stored_settings);
new_settings.c_cc[VMIN] = 1;
tcsetattr(0,TCSANOW,&new_settings);

in = getchar();

tcsetattr(0,TCSANOW,&stored_settings);
return in;
}

/* This example creates a subclass of Node and uses std::bind() to register a* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(): Node("minimal_publisher"), count_(0)
    {
        led0 = 0;
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("demo/cmd_demo", 10);
        publisher2 = this->create_publisher<std_msgs::msg::Int32>("led0", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        timer2 = this->create_wall_timer(2000ms, std::bind(&MinimalPublisher::led_test, this));

        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        //message.data = "Hello, world! " + std::to_string(count_++);
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = 0.0;

        char ch = scanKeyboard();
        if(ch == 'w')
            message.linear.x = 0.1;
        if(ch == 's')
            message.linear.x = -0.1;
        if(ch == 'a')
            message.angular.z = 0.1;
        if(ch == 'd')
            message.angular.z = -0.1;
        if(ch == ' ')
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
        }

        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    void led_test()
    {
        auto led = std_msgs::msg::Int32();
        led.data = led0;
        led0 = 1 - led0;
        publisher2->publish(led);
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_, timer2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher2;
    size_t count_;
    int led0;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
