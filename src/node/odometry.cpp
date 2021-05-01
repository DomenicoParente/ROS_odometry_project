//
// Created by Domenico Parente
//
#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "geometry_msgs/PoseStamped"
#include "robotics_hw1/MotorSpeed"
#include "nav_msgs/Odometry.h"
#include "string.h"
#include "math.h"

//Constants
const float WHEEL_RADIUS= 0.1575;
const float REAL_BASELINE= 0.583;
const double PI= 3.14159265;
const float GEAR_REDUCTION= 1/30;

// Initial position
double x=0.0;
double y=0.0;
double z=0.0;
double theta=0.0;
double t=0.0;

double RPM_converter(double rpm){
    return rpm*2*PI/60;
}

double Reduction_gearbox(double w){
    return w*GEAR_REDUCTION;
}

void odomCalculus(const robotics_hw1::MotorSpeed::ConstPtr& fl, const robotics_hw1::MotorSpeed::ConstPtr& fr, const robotics_hw1::MotorSpeed::ConstPtr& rl, const robotics_hw1::MotorSpeed::ConstPtr& rr,){
    double v_fl, v_fr, v_rl, v_rr;
    v_fl= GEAR_REDUCTION( RPM_converter(fl) ) * WHEEL_RADIUS;
    v_fr= GEAR_REDUCTION( RPM_converter(fr) ) * WHEEL_RADIUS;
    v_rl= GEAR_REDUCTION( RPM_converter(rl) ) * WHEEL_RADIUS;
    v_rr= GEAR_REDUCTION( RPM_converter(rr) ) * WHEEL_RADIUS;

    double xk;
    double yk;
    double Ts;

    //computation Ts
    if (t==0.0)
    {
        Ts=0.0;
    }
    else
    {
        Ts= Vl->header.stamp.toSec()- t;
    }
    t= fl->header.stamp.toSec();

    //computation Euler Integration
    if(){
        xk= x + Vx * Ts * cos(theta);
        yk= y + Vy * Ts * sin(theta);
        theta= theta_k + w * Ts;
    }
    //computation Runge-Kutta Integration
    else{
        xk= x + Vx * Ts * cos(theta_k + (w * Ts)/2 );
        yk= y + Vy * Ts * cos(theta_k + (w * Ts)/2 );
        theta= theta_k + w * Ts;
    }

    //publish custom odom message

}

void test_callback(const robotics_hw1::MotorSpeed::ConstPtr& fl, const robotics_hw1::MotorSpeed::ConstPtr& fr, const robotics_hw1::MotorSpeed::ConstPtr& rl, const robotics_hw1::MotorSpeed::ConstPtr& rr,){
    ROS_INFO("%s %f - %s %f \n %s %f - %s %f", fl->header, fl->rpm, fr->header, fr->rpm, rl->header, rl->rpm, rr->header, rr->rpm);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry_node");

    ros::NodeHandle n;
    ROS_INFO("STATE: OK");

    message_filters::Subscriber<geometry_msgs::PoseStamped> Pose(n, "gt_pose", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_fl(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_fr(n, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_rl(n, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_rr(n, "motor_speed_rr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> Odom(n, "scout_odom", 1);

    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed> sync(MS_fl, MS_fr, MS_rl, MS_rr, 10);

    sync.registerCallback(boost::bind(&test_callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}