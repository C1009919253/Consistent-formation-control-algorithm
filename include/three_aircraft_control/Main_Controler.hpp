#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termio.h>
#include <stdio.h>
#include <cmath>
#include <ct/core/core.h>
#include <ct/core/control/continuous_time/siso/PIDController.h>

#include "three_aircraft_control/msg/offsets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define pie 3.1415926
// 三机控制代码（当然你可以加几个机）

using namespace std::chrono_literals;
//using namespace std::placeholders; //_1 与ct冲突

class Main_Controler : public rclcpp::Node
{
public:
  Main_Controler();


private:
    void timer1_callback(); // for some test?
    void timer2_callback(); // 作为robot1,2,3的控制器

    void topic1_callback(nav_msgs::msg::Odometry::SharedPtr odom); // 注意是指针
    void topic2_callback(nav_msgs::msg::Odometry::SharedPtr odom);
    void topic3_callback(nav_msgs::msg::Odometry::SharedPtr odom);

    void topic4_callback(three_aircraft_control::msg::Offsets::SharedPtr offsets); // control the formation, NOTE: gobal coordinate system

    void topic_callback_test(geometry_msgs::msg::Quaternion::SharedPtr tesst); // 接收位置信息

    rclcpp::TimerBase::SharedPtr timer1, timer2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1, publisher2, publisher3; // robots' publishers
    geometry_msgs::msg::Twist twist1, twist2, twist3; // robots' twists
    nav_msgs::msg::Odometry odom1, odom2, odom3; // robots' pose (include Quaternion and Pose)

    //double gx, gy, gz, gYaw;

    double roll1, pitch1, yaw1; // robots' rotation
    double roll2, pitch2, yaw2;
    double roll3, pitch3, yaw3;

    double x1, y1, z1; //  robots' position
    double x2, y2, z2;
    double x3, y3, z3;

    double last12,last13; // remember this

    double offsetx12, offsety12; // the offset between robots
    double offsetx22, offsety22;
    double offsetx13, offsety13;
    double offsetx33, offsety33;

    size_t count_;

    geometry_msgs::msg::Quaternion goal_location;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1, subscription2, subscription3; // robots' subscription

    rclcpp::Subscription<three_aircraft_control::msg::Offsets>::SharedPtr subscription4; // control the formation

    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscription_test; // for a test...

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_test;

    bool test; // for test, do not use it expect you are me...

    ct::core::PIDController<double>::parameters_t params;
    ct::core::PIDController<double>::setpoint_t setpoints;
    ct::core::PIDController<double> pid;

    ct::core::Time current_time;

};
