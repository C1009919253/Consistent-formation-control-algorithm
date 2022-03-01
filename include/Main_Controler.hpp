#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termio.h>
#include <stdio.h>
#include <cmath>
#include <ct/core/core.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    Main_Controler(): Node("Main_Controler"), count_(0)
    {
        twist1.linear.x = 0.0;
        twist1.linear.y = 0.0;
        twist1.linear.z = 0.0;
        twist1.angular.x = 0.0;
        twist1.angular.y = 0.0;
        twist1.angular.z = 0.0;

        twist2.linear.x = 0.0;
        twist2.linear.y = 0.0;
        twist2.linear.z = 0.0;
        twist2.angular.x = 0.0;
        twist2.angular.y = 0.0;
        twist2.angular.z = 0.0;

        twist3.linear.x = 0.0;
        twist3.linear.y = 0.0;
        twist3.linear.z = 0.0;
        twist3.angular.x = 0.0;
        twist3.angular.y = 0.0;
        twist3.angular.z = 0.0;

        publisher1 = this->create_publisher<geometry_msgs::msg::Twist>("demo1/cmd_demo", 10);
        publisher2 = this->create_publisher<geometry_msgs::msg::Twist>("demo2/cmd_demo", 10);
        publisher3 = this->create_publisher<geometry_msgs::msg::Twist>("demo3/cmd_demo", 10);
        timer1 = this->create_wall_timer(50ms, std::bind(&Main_Controler::timer1_callback, this));
        timer2 = this->create_wall_timer(50ms, std::bind(&Main_Controler::timer2_callback, this));

        subscription1 = this->create_subscription<nav_msgs::msg::Odometry>("demo1/odom_demo", 10, std::bind(&Main_Controler::topic1_callback, this, std::placeholders::_1));
        subscription2 = this->create_subscription<nav_msgs::msg::Odometry>("demo2/odom_demo", 10, std::bind(&Main_Controler::topic2_callback, this, std::placeholders::_1));
        subscription3 = this->create_subscription<nav_msgs::msg::Odometry>("demo3/odom_demo", 10, std::bind(&Main_Controler::topic3_callback, this, std::placeholders::_1));

        offsetx12 = -20;
        offsety12 = 0;
        offsetx22 = 0;
        offsety22 = 0;
        offsetx13 = -40;
        offsety13 = 0;
        offsetx33 = 0;
        offsety33 = 0;

    }

private:
    void timer1_callback() // put your code here!
    {
        auto message = geometry_msgs::msg::Twist();
        //message.data = "Hello, world! " + std::to_string(count_++);
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = 0.0;

        //char ch = scanKeyboard();
        char ch = 'g';
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
        //publisher1->publish(message);
        //publisher2->publish(message);
        //publisher3->publish(message);
    }

    void timer2_callback() // 作为robot1,2,3的控制器
    {
        //tf2::Quaternion imu_quat = odom1.pose.pose.orientation;
        //double distence = (gx - odom1.pose.pose.position.x) * (gx - odom1.pose.pose.position.x) + (gy - odom1.pose.pose.position.y) * (gy - odom1.pose.pose.position.y);
        //double v = 2 - 200 / (distence + 100);
        twist1.linear.x = 1;
        publisher1->publish(twist1); // leader run with no eye

        /*double v2 = 1 * (cos(yaw2)*(odom1.pose.pose.position.x-odom2.pose.pose.position.x+offsetx12*cos(yaw1)-offsety12*sin(yaw1))+sin(yaw2)*(odom1.pose.pose.position.y-odom2.pose.pose.position.y+offsety12*cos(yaw1)+offsetx12*sin(yaw1)));
        twist2.linear.x = v2;
        //twist2.linear.x = 0.5 * sqrt(pow(odom1.pose.pose.position.x-odom2.pose.pose.position.x-10,2)+pow(odom1.pose.pose.position.y-odom2.pose.pose.position.y+0,2));
        double ang = atan2(odom1.pose.pose.position.y-odom2.pose.pose.position.y+offsety12*cos(yaw1)+offsetx12*sin(yaw1), odom1.pose.pose.position.x-odom2.pose.pose.position.x+offsetx12*cos(yaw1)-offsety12*sin(yaw1));
        twist2.angular.z = 0.9 * twist2.angular.z - 1 * (ang - 0);
        last12 = ang;
        publisher2->publish(twist2);

        double v3 = 1 * (cos(yaw3)*(odom1.pose.pose.position.x-odom3.pose.pose.position.x+offsetx13*cos(yaw1)-offsety13*sin(yaw1))+sin(yaw3)*(odom1.pose.pose.position.y-odom3.pose.pose.position.y+offsety13*cos(yaw1)+offsetx13*sin(yaw1)));
        twist3.linear.x = v3;
        //twist2.linear.x = 0.5 * sqrt(pow(odom1.pose.pose.position.x-odom2.pose.pose.position.x-10,2)+pow(odom1.pose.pose.position.y-odom2.pose.pose.position.y+0,2));
        double ang2 = atan2(odom1.pose.pose.position.y-odom3.pose.pose.position.y+offsety13*cos(yaw1)+offsetx13*sin(yaw1), odom1.pose.pose.position.x-odom3.pose.pose.position.x+offsetx13*cos(yaw1)-offsety13*sin(yaw1));
        twist3.angular.z = 0.9 * twist3.angular.z - 4 * (ang2 - last13);
        last13 = ang2;
        publisher3->publish(twist3);*/

        double vx2 = -0.25 * ((x2-x1-offsetx12)+offsetx22); // offsets are all in reference coordinate system, to change them you maight compute tem
        double vy2 = -0.25 * ((y2-y1-offsety12)+offsety22);

        double theta12 = atan2(vy2, vx2);
        double v2 = vx2 / cos(theta12); // ➗0bug
        double w2 = theta12 - yaw2;

        if (abs(w2) > pie / 2)
        {
            v2 = -v2;
            if (w2 > 0)
                w2 = w2 - pie;
            else
                w2 = pie + w2;
        }

        twist2.linear.x = v2;
        twist2.angular.z = w2*0.25;
        publisher2->publish(twist2);

        double vx3 = -0.5 * ((x3-x1-offsetx13)+offsetx33);
        double vy3 = -0.5 * ((y3-y1-offsety13)+offsety33);

        double theta13 = atan2(vy3, vx3);
        double v3 = vx3 / cos(theta13);
        double w3 = theta13 - yaw3;

        if (abs(w3) > pie / 2)
        {
            v3 = -v3;
            if (w3 > 0)
                w3 = w3 - pie;
            else
                w3 = pie + w3;
        }

        twist3.linear.x = v3;
        twist3.angular.z = w3*0.25;
        publisher3->publish(twist3);

    }

    void topic1_callback(nav_msgs::msg::Odometry::SharedPtr odom) // 注意是指针
    {
        odom1 = *odom;
        tf2::Quaternion imu(odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w);
        tf2::Matrix3x3 m(imu);
        m.getRPY(roll1, pitch1, yaw1);

        x1 = odom1.pose.pose.position.x;
        y1 = odom1.pose.pose.position.y;
        z1 = odom1.pose.pose.position.z;

    }

    void topic2_callback(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        odom2 = *odom;
        tf2::Quaternion imu(odom2.pose.pose.orientation.x, odom2.pose.pose.orientation.y, odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w);
        tf2::Matrix3x3 m(imu);
        m.getRPY(roll2, pitch2, yaw2);

        x2 = odom2.pose.pose.position.x;
        y2 = odom2.pose.pose.position.y;
        z2 = odom2.pose.pose.position.z;

    }

    void topic3_callback(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        odom3 = *odom;
        tf2::Quaternion imu(odom3.pose.pose.orientation.x, odom3.pose.pose.orientation.y, odom3.pose.pose.orientation.z, odom3.pose.pose.orientation.w);
        tf2::Matrix3x3 m(imu);
        m.getRPY(roll3, pitch3, yaw3);

        x3 = odom3.pose.pose.position.x;
        y3 = odom3.pose.pose.position.y;
        z3 = odom3.pose.pose.position.z;

    }

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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1, subscription2, subscription3; // robots' subscription
};
