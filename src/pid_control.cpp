#include "gazebo_control/control.hpp"

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control>("pid_control_node");
    rclcpp::spin(node);
    rclcpp::shutdown();

}