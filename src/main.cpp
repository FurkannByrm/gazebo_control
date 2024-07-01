#include "gazebo_control/pid_control.hpp"

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();

}