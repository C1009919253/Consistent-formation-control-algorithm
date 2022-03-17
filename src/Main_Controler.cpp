#include "three_aircraft_control/Main_Controler.hpp"

// Adapted from
// https://github.com/JunshengFu/Model-Predictive-Control/tree/master/src/main.cpp

void map2car(double px, double py, double psi, const vector<double>& ptsx_map, const vector<double>& ptsy_map,
             Eigen::VectorXd & ptsx_car, Eigen::VectorXd & ptsy_car){

  for(size_t i=0; i< ptsx_map.size(); i++){
    double dx = ptsx_map[i] - px;
    double dy = ptsy_map[i] - py;
    ptsx_car[i] = dx * cos(-psi) - dy * sin(-psi);
    ptsy_car[i] = dx * sin(-psi) + dy * cos(-psi);
    /*ptsx_car[i] = ptsx_map[i];
    ptsy_car[i] = ptsy_map[i];*/
  }
}

// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
  /*Eigen::VectorXd result(xvals.size());
  for (int i = 0; i < xvals.size(); i++) {
      result(i) = 1;
  }
  return result;*/
}


Main_Controler::Main_Controler(): Node("Main_Controler"), count_(0)
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

    publisher_test = this->create_publisher<geometry_msgs::msg::Twist>("Vehicle3/cmd_vel", 10);

    timer1 = this->create_wall_timer(1000ms, std::bind(&Main_Controler::timer1_callback, this));
    timer2 = this->create_wall_timer(50ms, std::bind(&Main_Controler::timer2_callback, this));
    timer3 = this->create_wall_timer(50ms, std::bind(&Main_Controler::timer3_callback, this));

    subscription1 = this->create_subscription<nav_msgs::msg::Odometry>("demo1/odom_demo", 10, std::bind(&Main_Controler::topic1_callback, this, std::placeholders::_1));
    subscription2 = this->create_subscription<nav_msgs::msg::Odometry>("demo2/odom_demo", 10, std::bind(&Main_Controler::topic2_callback, this, std::placeholders::_1));
    subscription3 = this->create_subscription<nav_msgs::msg::Odometry>("demo3/odom_demo", 10, std::bind(&Main_Controler::topic3_callback, this, std::placeholders::_1));

    subscription4 = this->create_subscription<three_aircraft_control::msg::Offsets>("MC/set_offsets", 10, std::bind(&Main_Controler::topic4_callback, this, std::placeholders::_1));

    subscription_test = this->create_subscription<geometry_msgs::msg::Quaternion>("Vehicle3/object_topic", 10, std::bind(&Main_Controler::topic_callback_test, this, std::placeholders::_1));

    offsetx12 = -10;
    offsety12 = 10;
    offsetx22 = 0;
    offsety22 = 0;
    offsetx13 = -10;
    offsety13 = -10;
    offsetx33 = 0;
    offsety33 = 0;

    test = true;

    params.k_p = 1;
    params.k_i = 0.15;
    params.k_d = 0.00;

    pid = ct::core::PIDController<double>(params, setpoints);

    current_time = 0;

    //x0.setZero();
    //start_time = std::chrono::high_resolution_clock::now();
}

void Main_Controler::timer1_callback() // test~
{
    geometry_msgs::msg::Twist twist_test;
    //twist_test.linear.x = (test) ? 0.1 : 0;
    //test = false;

    double x_g, y_g;
    x_g = goal_location.y;
    y_g = -goal_location.x;

    double vx = -0.25 * ((x_g+0.5)+0); // offsets are all in reference coordinate system, to change them you maight compute tem
    double vy = -0.25 * ((y_g-0)+0);

    double theta = atan2(vy, vx);
    double v = (abs(cos(theta)) > abs(sin(theta))) ? vx / cos(theta) : vy / sin(theta);
    double w = theta;

    if (abs(w) > pie / 2)
    {
        v = -v;
        if (w > 0)
            w = w - pie;
        else
            w = pie + w;
    }

    double v_m = 0.2;

    v = (abs(v) < v_m) ? v : v/abs(v)*v_m;
    v = (goal_location.w) ? v : 0;
    twist_test.linear.x = -v;
    twist_test.angular.z = w;

    publisher_test->publish(twist_test);

}

void Main_Controler::timer2_callback()
{
    twist1.linear.x = 1;
    publisher1->publish(twist1); // leader run with no eye

    double vx2 = -0.25 * ((x2-x1-offsetx12)+offsetx22); // offsets are all in reference coordinate system, to change them you maight compute tem
    double vy2 = -0.25 * ((y2-y1-offsety12)+offsety22);
    double theta12 = atan2(vy2, vx2);
    double pv2 = (abs(cos(theta12)) > abs(sin(theta12))) ? vx2 / cos(theta12) : vy2 / sin(theta12); // ➗0bug fixed
    double pw2 = theta12 - yaw2;

    if (abs(pw2) > pie / 2)
    {
        pv2 = -pv2;
        if (pw2 > 0)
            pw2 = pw2 - pie;
        else
            pw2 = pie + pw2;
    }

    //pv2 = -pid.computeControl(pv2, current_time);


    Eigen::VectorXd ptsx_car(ptsx2.size());
    Eigen::VectorXd ptsy_car(ptsy2.size());

    map2car(x2, y2, yaw2, ptsx2, ptsy2, ptsx_car, ptsy_car);

    if(ptsx_car.size() < 4)
        return;

    auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

    Eigen::VectorXd state(3);

    double v2 = odom2.twist.twist.linear.x;

    double px2 = 0 + v2 * cos(0) * 0.1;
    double py2 = 0 + v2 * sin(0) * 0.1;
    double psi2 = 0 - odom2.twist.twist.angular.z;

    state << px2, py2, psi2;

    auto vars = mpc.Solve(state, coeffs, pv2, -pw2);

    double w2 = -vars[0];
    v2 = vars[1];

    //v2 = -pid.computeControl(v2, current_time);


    /*v2 = (abs(v2) < 2) ? v2 : v2/abs(v2)*2;

    w2 = (abs(w2) < 2) ? w2 : w2/abs(w2)*2;*/

    twist2.linear.x = v2;
    twist2.angular.z = w2;
    publisher2->publish(twist2);

    ptsx2.clear();
    ptsy2.clear();

    double vx3 = -0.5 * ((x3-x1-offsetx13)+offsetx33);
    double vy3 = -0.5 * ((y3-y1-offsety13)+offsety33);



    double theta13 = atan2(vy3, vx3);
    double v3 = (abs(cos(theta13)) > abs(sin(theta13))) ? vx3 / cos(theta13) : vy3 / sin(theta13);
    double w3 = theta13 - yaw3;

    if (abs(w3) > pie / 2)
    {
        v3 = -v3;
        if (w3 > 0)
            w3 = w3 - pie;
        else
            w3 = pie + w3;
    }

    v3 = (abs(v3) < 2) ? v3 : v3/abs(v3)*2;

    twist3.linear.x = v3;
    twist3.angular.z = w3*1;
    publisher3->publish(twist3);

    current_time += 0.05;

}

void Main_Controler::timer3_callback()
{

}

void Main_Controler::topic1_callback(nav_msgs::msg::Odometry::SharedPtr odom)
{//RCLCPP_INFO(this->get_logger(), "12!\n");
    odom1 = *odom;
    tf2::Quaternion imu(odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w);
    tf2::Matrix3x3 m(imu);
    m.getRPY(roll1, pitch1, yaw1);

    x1 = odom1.pose.pose.position.x;
    y1 = odom1.pose.pose.position.y;
    z1 = odom1.pose.pose.position.z;

    ptsx2.push_back(x1 + offsetx12);
    ptsy2.push_back(y1 + offsety12);

}

void Main_Controler::topic2_callback(nav_msgs::msg::Odometry::SharedPtr odom)
{
    odom2 = *odom;
    tf2::Quaternion imu(odom2.pose.pose.orientation.x, odom2.pose.pose.orientation.y, odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w);
    tf2::Matrix3x3 m(imu);
    m.getRPY(roll2, pitch2, yaw2);

    x2 = odom2.pose.pose.position.x;
    y2 = odom2.pose.pose.position.y;
    z2 = odom2.pose.pose.position.z;

}

void Main_Controler::topic3_callback(nav_msgs::msg::Odometry::SharedPtr odom)
{
    odom3 = *odom;
    tf2::Quaternion imu(odom3.pose.pose.orientation.x, odom3.pose.pose.orientation.y, odom3.pose.pose.orientation.z, odom3.pose.pose.orientation.w);
    tf2::Matrix3x3 m(imu);
    m.getRPY(roll3, pitch3, yaw3);

    x3 = odom3.pose.pose.position.x;
    y3 = odom3.pose.pose.position.y;
    z3 = odom3.pose.pose.position.z;

}

void Main_Controler::topic4_callback(three_aircraft_control::msg::Offsets::SharedPtr offsets)
{   //RCLCPP_INFO(this->get_logger(), "1!\n");
    three_aircraft_control::msg::Offsets offsetxx = *offsets;

    offsetx12 = offsetxx.offsetx12;
    offsetx13 = offsetxx.offsetx13;
    offsetx22 = offsetxx.offsetx22;
    offsetx33 = offsetxx.offsetx33;

    offsety12 = offsetxx.offsety12;
    offsety13 = offsetxx.offsety13;
    offsety22 = offsetxx.offsety22;
    offsety33 = offsetxx.offsety33;

}

void Main_Controler::topic_callback_test(geometry_msgs::msg::Quaternion::SharedPtr tesst)
{
    goal_location = *tesst;
    //printf("%f, %f, %f, %f", goal_location.x, goal_location.y, goal_location.z, goal_location.w);
}
