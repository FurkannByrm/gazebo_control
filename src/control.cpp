#include "gazebo_control/control.hpp"

void Control::pidAlgorthm(){

    double linear_error;
    double angle_error;
    double linear_velocity;
    double angle_velocity;

    //update the PID-releated error states
    error_forward = scan_range - target_distance;
    error_angle_ =scan_angle - target_angle;

    error_angle_ = normalizeAngle(error_angle_);
    
    P_angle_     = kp_a * error_angle_;
    P_forward_   = kp_a * error_forward;

    I_angle_      += error_angle_ * dt;
    I_forward_    += error_forward *dt;

    D_angle_      = kd_a * normalizeAngle(error_angle_ - error_angle_prev_) / dt;
    D_forward_    = kd_f * (error_forward - error_angle_prev_) / dt;

    trans_forward_ = P_forward_ + ki_f * I_forward_ +D_angle_;
    trans_angle_   = -(P_angle_ + ki_a*I_angle_ + D_angle_);

    error_angle_prev_ = error_forward;
    error_angle_prev_ = error_angle_;

    trans_forward_ = std::max(0.0, std::min(trans_forward_, 7.0)); 


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