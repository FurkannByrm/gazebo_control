#ifndef PID_CONTROL_
#define PID_CONTROL_


#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

constexpr double PI = 3.1415;


class Control: public rclcpp::Node{

public:

    double dt;
    double target_distance, target_angle_;
    double kp_f, ki_f, kd_f;
    double kp_a, ki_a, kd_a;

    Control();
    ~Control() = default;

private:
    std_msgs::msg::Float32 linear_error;
    std_msgs::msg::Float32 angle_error;
    std_msgs::msg::Float32 linear_velocity;
    std_msgs::msg::Float32 angle_velocity;
    
    //topic to be subscribed
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    
    //topics to be published
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    error_forward_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    error_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    control_signal_forward_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    control_signal_angle_pub_;

    std::vector<float> scan_data_; //from laser scan
    float scan_range_;
    float scan_ang_;

    double pos_x_, pos_y_, q_z_; //from odometry, the robot moves flat, the robot will not turn along x axis or y axis
    double ang_z_; //eular angle from quaternion q_z

    geometry_msgs::msg::Twist vel_cmd_; //control the robot msgs
    double trans_forward_, trans_angle_; //pid output

    //PID- related states
    double error_forward_, error_angle_, error_forward_prev_, error_angle_prev_;
    double P_forward, P_angle_; // proportional part
    double I_forward, I_angle_; // integral part 
    double D_forward, D_angle_; // derivative part

    void odomCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr& odomMsg);
    void scanCallBack(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg);
    void pidAlgorthm();
    bool loadParam();

    double normalizeAngle(double angle);
    double init_x, init_y, init_ang;



};


#endif