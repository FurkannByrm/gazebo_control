#include "gazebo_control/pid_control.hpp"

Control::Control(): Node("sub_pub_nodes"){

    //load the param
    if(!loadParam())
    {
        RCLCPP_ERROR(this->get_logger(), "ERROR IN LOADING THE PARAMETERS.");
    }
    
    //declare all the subscriber and publisher 
    scan_sub_                       = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1 ,std::bind(&Control::scanCallBack, this));
    odom_sub_                       = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&Control::odomCallBack, this));

    vel_pub_                        = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 200);
    error_forward_pub_              = this->create_publisher<std_msgs::msg::Float32>("/error_forward", 1);
    error_angle_pub_                = this->create_publisher<std_msgs::msg::Float32>("/error_angle", 1);
    control_signal_forward_pub_     = this->create_publisher<std_msgs::msg::Float32>("/control_signal_forward", 1);
    control_signal_angle_pub_       = this->create_publisher<std_msgs::msg::Float32>("/control_signal_angle", 1);


    error_forward_                  = 0.0;
    error_forward_prev_             = 0.0;
    error_angle_                    = 0.0;
    error_angle_                    = 0.0;
    error_angle_prev_               = 0.0;
    I_angle_                        = 0.0;
    I_forward                       = 0.0;
    D_forward                       = 0.0;
    D_angle_                        = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "[NODE INITIALIZED]");

}

void Control::odomCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr& odomMsg){

    pos_x_ = odomMsg->pose.pose.position.x;
    pos_y_ = odomMsg->pose.pose.position.y;
    q_z_   = odomMsg->pose.pose.orientation.z;
    ang_z_ = ang_z_ * 2.19;
}
void Control::scanCallBack(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg){
    scan_data_ = scanMsg->ranges;
    int arr_size = scan_data_.size();
    float smallest_dist = 100;
    for (int i = 0; i <arr_size ; i++)
    {
        if (scan_data_[i] < smallest_dist)
        {
            smallest_dist = scan_data_[i];
            scan_ang_     = scanMsg->angle_min +scanMsg->angle_increment*i;  
        }
        
    }
    scan_range_ = smallest_dist;
    pidAlgorthm();
}

void Control::pidAlgorthm(){

    std_msgs::msg::Float32 linear_error;
    std_msgs::msg::Float32 angle_error;
    std_msgs::msg::Float32 linear_velocity;
    std_msgs::msg::Float32 angle_velocity;

    //update the PID-related error states
    error_forward_      = scan_range_ - target_distance;
    error_angle_        = scan_ang_   - target_angle_;

    //normalise the error_angle within [-PI, PI]
    error_angle_        = normalizeAngle(error_angle_);

    //define proportional term
    P_angle_            = kp_a * error_angle_;
    P_forward           = kp_f * error_forward_;

    //define integral term
    I_angle_            += error_angle_ * dt;
    I_forward           += error_forward_ * dt;

    //define derivative term
    D_angle_            = kd_a * normalizeAngle(error_angle_ - error_angle_prev_) / dt;
    D_forward           = kd_f * (error_forward_ - error_angle_prev_) / dt;

    //compute PID
    trans_forward_      = P_forward + ki_f * I_forward + D_forward;
    trans_angle_        = -(P_angle_ + kd_a * I_angle_ + D_angle_);

    error_forward_prev_ = error_forward_;
    error_angle_prev_   = error_angle_;

    //set threshold (optinal)
    trans_forward_      = std::max(0.0, std::min(trans_forward_, 7.0));

    //end of PID compuation
    RCLCPP_INFO(this->get_logger(), "forward_velocity: %f; Angle_velocity: %f; Orientation_error: %f; Distance: %f",trans_forward_, trans_angle_, error_angle_, scan_range_);

    //publish all




}

// #include "gazebo_control/control.hpp"

// int main(int argc, char ** argv){

//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<Control>("pid_control_node");
//     rclcpp::spin(node);
//     rclcpp::shutdown();

// }