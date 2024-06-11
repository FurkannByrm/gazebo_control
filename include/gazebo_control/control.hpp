#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include <iostream>
static const double PI = 3.1415;

class Control
{
private:


    void odomCallback(const double& encoder_data);
    void scanData(const double& scan_data );
    void pidAlgorthm();
    double normalizeAngle(double angle);
    
    float scan_range;
    float scan_angle;

    double  pos_x, pos_y, pos_z; //from odometry, the robot moves flat, the robot will not turn along x axis or y axis
    double ang_z; //eular angle from quaternion q_z

    double trans_forward_, trans_angle_;

    double error_forward, error_angle_, error_prev_, error_angle_prev_;
    double P_forward_, P_angle_; //proportional part
    double I_forward_, I_angle_;      //integral part
    double D_forward_, D_angle_;     //derivative part

    double normalizeAngle(double angle);
    double init_x, init_y, init_ang;
    
public:

    double dt;
    double target_distance, target_angle;
    double kp_f, ki_f, kd_f;
    double kp_a, ki_a, kd_a;
};

#endif