#include "three_aircraft_control/Main_Controler.hpp"

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
    params.k_i = 0.3;
    params.k_d = 0.05;

    pid = ct::core::PIDController<double>(params, setpoints);

    current_time = 0;

    ilqr_settings.dt = 0.01;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::
        GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings.printSummary = true;

    ilqr_settings_mpc.dt = 0.01;
    ilqr_settings_mpc.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings_mpc.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings_mpc.max_iterations = 1;
    ilqr_settings_mpc.nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings_mpc.lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
    ilqr_settings_mpc.printSummary = false;

    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
    mpc_settings.coldStart_ = false;

    //Car_Sim.reset(new sim_car_system<double>);
    x0.setZero();

    bool verbose = true;
    intermediateCost.reset(new ct::optcon::TermQuadratic<3, 2>); // useful?
    finalCost.reset(new ct::optcon::TermQuadratic<3, 2>());
    adLinearizer.reset(new ct::core::SystemLinearizer<3, 2>(Car_Sim));
    intermediateCost->loadConfigFile(ct::optcon::exampleDir + "/mpcCost.info", "intermediateCost", verbose);
    finalCost->loadConfigFile(ct::optcon::exampleDir + "/mpcCost.info", "finalCost", verbose);

    costFunction.reset(new ct::optcon::CostFunctionAnalytical<3, 2>());

    costFunction->addIntermediateTerm(intermediateCost);
    costFunction->addFinalTerm(finalCost);
    //costFunction->addIntermediateTerm(intermediateCost)

    optConProblem = ct::optcon::ContinuousOptConProblem<3, 2>(timeHorizon, x0, Car_Sim, costFunction, adLinearizer);

    ct::optcon::NLOptConSolver<3, 2> iLQR(optConProblem, ilqr_settings);//iLQR = ct::optcon::NLOptConSolver<3, 2>(optConProblem, ilqr_settings);

    size_t K = ilqr_settings.computeK(timeHorizon);

    ct::core::FeedbackArray<3, 2> u0_fb(K, ct::core::FeedbackMatrix<3, 2>::Zero());
    ct::core::ControlVectorArray<2> u0_ff(K, ct::core::ControlVector<2>::Zero());
    ct::core::StateVectorArray<3> x_ref_init(K + 1, x0);
    ct::optcon::NLOptConSolver<3, 2>::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

    iLQR.setInitialGuess(initController);
    iLQR.solve();
    ct::core::StateFeedbackController<3, 2> initialSolution = iLQR.getSolution();

    //ilqr_mpc = ct::optcon::MPC<ct::optcon::NLOptConSolver<3, 2>>(optConProblem, ilqr_settings_mpc, mpc_settings);

    static ct::optcon::MPC<ct::optcon::NLOptConSolver<3, 2>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);

    ilqr_mpc.setInitialGuess(initialSolution);

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
    double v2 = (abs(cos(theta12)) > abs(sin(theta12))) ? vx2 / cos(theta12) : vy2 / sin(theta12); // ➗0bug fixed
    double w2 = theta12 - yaw2;

    if (abs(w2) > pie / 2)
    {
        v2 = -v2;
        if (w2 > 0)
            w2 = w2 - pie;
        else
            w2 = pie + w2;
    }

    v2 = pid.computeControl(v2, current_time);

    v2 = (abs(v2) < 2) ? v2 : v2/abs(v2)*2;

    twist2.linear.x = -v2;
    twist2.angular.z = w2*1;
    publisher2->publish(twist2);

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

void Main_Controler::topic1_callback(nav_msgs::msg::Odometry::SharedPtr odom)
{
    odom1 = *odom;
    tf2::Quaternion imu(odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w);
    tf2::Matrix3x3 m(imu);
    m.getRPY(roll1, pitch1, yaw1);

    x1 = odom1.pose.pose.position.x;
    y1 = odom1.pose.pose.position.y;
    z1 = odom1.pose.pose.position.z;

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
{
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
