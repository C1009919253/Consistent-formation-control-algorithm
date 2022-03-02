#include "Main_Controler.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Main_Controler>());
    rclcpp::shutdown();
    return 0;
}
