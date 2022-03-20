#include <ct/optcon/optcon.h>
#include "three_aircraft_control/exampleDir.h"
#include "three_aircraft_control/sim_car_system.hpp"
#include "three_aircraft_control/Distributed_Controller.hpp"
//#include "three_aircraft_control/my_mpc.hpp"

using namespace ct::core;
using namespace ct::optcon;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<Main_Controler>());
    rclcpp::spin(std::make_shared<Distributed_Controller>());
    rclcpp::shutdown();
    return 0;
}
