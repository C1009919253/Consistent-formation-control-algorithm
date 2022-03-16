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

/*void Main_Controler::create_controller(const ct::core::StateVector<3>& x_init, const ct::core::StateVector<3>& x_ref)
{
    // Step 1: setup Nonlinear Optimal Control Problem

    const size_t state_dim = 3;
    const size_t control_dim = 2;

    double w_n = 0.1;
    double zeta = 5.0;
    double g_dc = 1.0;

    // Step 1-A: create controller instance

    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> oscillatorDynamics(
        new TAC::CAR::sim_car_system<double>(w_n, zeta));

    // Step 1-B: create a numerical linearizer

    std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(new
        SystemLinearizer<state_dim, control_dim>(oscillatorDynamics)
    );


    // STEP 1-C: create a cost function.
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    this->termQuad_interm = intermediateCost;
    this->termQuad_final = finalCost;

    this->termQuad_interm->loadConfigFile(exampleDir + "/mpcCost.info", "intermediateCost");
    this->termQuad_final->loadConfigFile(exampleDir + "/mpcCost.info", "finalCost");
    this->termQuad_interm->updateReferenceState(this->x_ref);
    this->termQuad_final->updateReferenceState(this->x_ref);


    CostFuncPtr_t costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());
    this->costFunc = costFunction;

    this->costFunc->addIntermediateTerm(this->termQuad_interm);
    this->costFunc->addFinalTerm(this->termQuad_final);

    // Check which cost function should I use here.

    // STEP 1-D: initialization with initial state and desired time horizon

    ct::core::Time timeHorizon = 3.0;

    // STEP 1-E: create and initialize an "optimal control problem"
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        timeHorizon, x_init, oscillatorDynamics, this->costFunc, adLinearizer);

    // STEP 2-A: Create the settings.

    NLOptConSettings nloc_settings;
    nloc_settings.load(exampleDir + "/lqrCost.info", true, "ilqr");

    // STEP 2-B: provide an initial guess
    N = nloc_settings.computeK(timeHorizon);
    FeedbackMatrix<state_dim, control_dim> u_fb;
    u_fb << 0, 0, 0, 0, 0, 0;
    FeedbackArray<state_dim, control_dim> u0_fb(N, -u_fb);
    ControlVectorArray<control_dim> u0_ff(N, ControlVector<control_dim>::Zero());
    StateVectorArray<state_dim> x_ref_init(N + 1, x_ref);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);
    // How to create a more complicated controller for iterations?

    // STEP 2-C: create an NLOptConSolver instance
    //NLOPPtr_t iLQR(new NLOptConSolver<state_dim, control_dim>(optConProblem, nloc_settings));
    NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, nloc_settings);
    //this->nlop_problem = iLQR;

    //this->nlop_problem->setInitialGuess(initController);
    iLQR.setInitialGuess(initController);

    // STEP 3: solve the optimal control problem
    //this->nlop_problem->solve();
    iLQR.solve();

    // STEP 4: retrieve the solution

    // plotResultsOscillator<state_dim, control_dim>(x_ref_init,
    //                                             u0_fb,
    //                                             u0_ff,
    //                                             TimeArray(N + 1, timeHorizon));

    //ct::core::StateFeedbackController<state_dim, control_dim> solution = this->nlop_problem->getSolution();
    ct::core::StateFeedbackController<state_dim, control_dim> solution = iLQR.getSolution();

    /*  MPC-EXAMPLE
    * we store the initial solution obtained from solving the initial optimal control problem,
    * and re-use it to initialize the MPC solver in the following. */
    /* STEP 1: first, we set up an MPC instance for the iLQR solver and configure it. Since the MPC
    * class is wrapped around normal Optimal Control Solvers, we need to different kind of settings,
    * those for the optimal control solver, and those specific to MPC: */
    // 1) settings for the iLQR instance used in MPC. Of course, we use the same settings
    // as for solving the initial problem ...
    /*NLOptConSettings ilqr_settings_mpc = nloc_settings;
    // ... however, in MPC-mode, it makes sense to limit the overall number of iLQR iterations (real-time iteration scheme)
    ilqr_settings_mpc.max_iterations = 1;
    // and we limited the printouts, too.
    ilqr_settings_mpc.printSummary = true;
    // 2) settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
    ct::optcon::mpc_settings mpc_settings;
    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
    mpc_settings.coldStart_ = false;
    // STEP 2 : Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
    MPCPtr_t ilqr_mpc(new MPC<NLOptConSolver<state_dim, control_dim>>(optConProblem, ilqr_settings_mpc, mpc_settings));

    // initialize it using the previously computed initial controller
    this->mpc = ilqr_mpc;
    this->mpc->setInitialGuess(solution);
}*/

/*void Main_Controler::pos_message_converter(const nav_msgs::msg::Odometry odom, ct::core::StateVector<3>& x)
{
    x(0) = odom.pose.pose.position.x;
    x(1) = odom.pose.pose.position.y;

    double roll, pitch, yaw;

    tf2::Quaternion imu(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(imu);
    m.getRPY(roll, pitch, yaw);

    x(2) = yaw;

}*/

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

    /*double vx2 = -0.25 * ((x2-x1-offsetx12)+offsetx22); // offsets are all in reference coordinate system, to change them you maight compute tem
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

    auto current_time = std::chrono::high_resolution_clock::now();
    ct::core::Time t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
    ilqr_mpc.prepareIteration(t);


    ct::core::Time ts_newPolicy;

    current_time = std::chrono::high_resolution_clock::now();
    t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

    bool success = ilqr_mpc.finishIteration(x2, t, newPolicy, ts_newPolicy);

    if (ilqr_mpc.timeHorizonReached() | !success)
    {
        RCLCPP_INFO(this->get_logger(), "Something error!\n");
    }

    ControlVector<2> u;

    newPolicy.computeControl(x2, t-ts_newPolicy,u);

    v2 = u(0);
    w2 = u(1);


    //v2 = pid.computeControl(v2, current_time);

    v2 = (abs(v2) < 2) ? v2 : v2/abs(v2)*2;

    twist2.linear.x = -v2;
    twist2.angular.z = w2*1;
    publisher2->publish(twist2);*/

    /*if (controller_not_created_)
        return; // Only start to publish when the mpc controller is created

    const size_t state_dim = 3;
    const size_t control_dim = 2;

    current_time = std::chrono::high_resolution_clock::now();
    ct::core::Time t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
    this->mpc->prepareIteration(t);
    current_time = std::chrono::high_resolution_clock::now();
    t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
    this->mpc->finishIteration(this->x_now, t, newPolicy, ts_newPolicy);
    current_time = std::chrono::high_resolution_clock::now();
    t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
    ControlVector<control_dim> u;
    newPolicy.computeControl(this->x_now, t - ts_newPolicy, u);

    double v2 = u(0)/10;
    double w2 = u(1)/100;

    RCLCPP_INFO(this->get_logger(), "output:%f,%f\n", v2, w2);*/

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
    /*nav_msgs::msg::Odometry msg;
    msg.pose.pose.position.x = x1 - offsetx12 - x_now(0);
    msg.pose.pose.position.y = y1 - offsety12 - x_now(1);

    RCLCPP_INFO(this->get_logger(), "X_REF:%f,%f\n",msg.pose.pose.position.x,msg.pose.pose.position.y);

    if (controller_not_created_)
    {
        if (first_pass_2)
            return;

        Main_Controler::pos_message_converter(msg, this->x_ref);
        Main_Controler::create_controller(this->x_now, this->x_ref);
        x_ref_current = this->x_ref;
        start_time = std::chrono::high_resolution_clock::now();
        controller_not_created_ = false;
        return;
    }
    Main_Controler::pos_message_converter(msg, this->x_ref);

    if (x_ref_current == this->x_ref)
        return;
    else
    {//RCLCPP_INFO(this->get_logger(), "change!\n");
        // Main_Controler::create_controller(this->x_now, this->x_ref);
        this->termQuad_interm->updateReferenceState(this->x_ref);
        this->termQuad_final->updateReferenceState(this->x_ref);
        this->costFunc->addIntermediateTerm(this->termQuad_interm);
        this->costFunc->addFinalTerm(this->termQuad_final);
        auto solver = this->mpc->getSolver();
        solver.changeCostFunction(this->costFunc);

        x_ref_current = this->x_ref;
        start_time = std::chrono::high_resolution_clock::now();
    }*/
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



    /*if (first_pass_2)
    {
        Main_Controler::pos_message_converter(*odom, this->x_now);
        first_pass_2 = false;
        //RCLCPP_INFO(this->get_logger(), "1!\n");
        return;
    }
    if (controller_not_created_)
        return;

    Main_Controler::pos_message_converter(*odom, this->x_now);
    RCLCPP_INFO(this->get_logger(), "X_NOW:%f,%f\n",x_now(0),x_now(1));*/

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
