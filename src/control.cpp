#include "gazebo_control/control.hpp"

Control::Control(): Node("sub_pub_nodes"){

using namespace std::placeholders;

if(!loadParam())
{
    RCLCPP_ERROR(this->get_logger(),"ERROR IN LOADING THE PARAMETERS.");
}
// declare all the subscriber and publisher
scan_sub_                  = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&Control::scanCallback, this));
odom_sub_                  = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&Control::odomCallback, this));
vel_pub_                   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 200);
error_forward_pub_         = this->create_publisher<std_msgs::msg::Float32>("/error_forward", 1);
error_angle_pub_           = this->create_publisher<std_msgs::msg::Float32>("/error_angle", 1);
control_signal_forward_pub_= this->create_publisher<std_msgs::msg::Float32>("/control_signal_forward", 1);
control_signal_angle_pub_  = this->create_publisher<std_msgs::msg::Float32>("/control_signal_angle_pub", 1);

//initialize variables
//enter your code here to initialise
error_forward_             = 0.0;
error_forward_prev_        = 0.0;
error_angle_               = 0.0;
error_angle_prev_          = 0.0;
I_angle_                   = 0.0;
I_forward_                 = 0.0;
I_angle_                   = 0.0;
D_forward_                 = 0.0;
D_angle_                   = 0.0;

RCLCPP_INFO(this->get_logger(),"NODE INITIALIZED");

}

void Control::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odomMsg){

    pos_x = odomMsg->pose.pose.position.x;
    pos_y = odomMsg->pose.pose.position.y;
    q_z   = odomMsg->pose.pose.orientation.z;
    ang_z = q_z * 2.19;
}
void Control::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg){

    scan_data_          = scanMsg->ranges;
    int arr_size        = scan_data_.size();
    float smallest_dist = 100;

    for(int i = 0; i<arr_size; i++)
    {
        if(scan_data_[i] < smallest_dist)
        {
            smallest_dist = scan_data_[i];
            scan_ang_ = scanMsg->angle_min + scanMsg->angle_increment * i;
            
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

    //update the PID-releated error states
    error_forward_      = scan_range_ - target_distance;
    error_angle_        = scan_ang_ - target_angle;

	// normalise the error_angle_ within [-PI, PI]
    error_angle_        = normalizeAngle(error_angle_);
    
	//define proportional term    
    P_angle_            = kp_a * error_angle_;
    P_forward_          = kp_a * error_forward_;

    // define integral term
    I_angle_            += error_angle_ * dt;
    I_forward_          += error_forward_ *dt;

	// define derivative term
    D_angle_            = kd_a * normalizeAngle(error_angle_ - error_angle_prev_) / dt;
    D_forward_          = kd_f * (error_forward_ - error_forward_prev_) / dt;
	
    // compute PID
    trans_forward_      = P_forward_ + ki_f * I_forward_ +D_angle_;
    trans_angle_        = -(P_angle_ + ki_a*I_angle_ + D_angle_);

    error_forward_prev_ = error_forward_;
    error_angle_prev_   = error_angle_;

	// set threshold (optional)
    trans_forward_      = std::max(0.0, std::min(trans_forward_, 7.0)); 

    RCLCPP_INFO(this->get_logger(),"Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Distance: %f", 
		trans_forward_, trans_angle_, error_angle_, scan_range_);
    
    vel_cmd_.linear.x  = trans_forward_;
    vel_cmd_.angular.z = trans_angle_;
    vel_pub_->publish(vel_cmd_);
    
    linear_error.data = error_forward_;
    error_forward_pub_->publish(linear_error);

    angle_error.data = error_angle_;
    error_angle_pub_->publish(angle_error);

    angle_velocity.data = trans_angle_;
    control_signal_angle_pub_->publish(angle_velocity);

}

double Control::normalizeAngle(double angle){
    if(angle >PI)
    {
        angle -= 2*PI;
        return normalizeAngle(angle);
    }
    else if(angle < -PI){
        angle += 2*PI;
        return normalizeAngle(angle);
    }
    return angle;
}