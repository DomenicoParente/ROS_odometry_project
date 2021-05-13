//
// Created by Domenico Parente
//

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "robotics_hw1/MotorSpeed.h"
#include "nav_msgs/Odometry.h"
#include "string.h"
#include "math.h"

//Constants
const float WHEEL_RADIUS= 0.1575;
const float REAL_BASELINE= 0.583;
const float APPARENT_BASELINE= 1.034602;
const double PI= 3.14159265;
const double GEAR_REDUCTION= 0.026740;

int i=0;
double mean=0;

ros::Publisher velocities_pub;

double RPM_converter(double rpm){
    return rpm*2*PI/60;
}

double Reduction_gearbox(double w){
    return w*GEAR_REDUCTION;
}

void velocities_Calculus(const robotics_hw1::MotorSpeedConstPtr& fl,
                  const robotics_hw1::MotorSpeedConstPtr& fr,
                  const robotics_hw1::MotorSpeedConstPtr& rl,
                  const robotics_hw1::MotorSpeedConstPtr& rr){
    double v_fl, v_fr, v_rl, v_rr;
    double  Vr,Vl;
    double Vk,Wz;
    v_fl= - Reduction_gearbox( RPM_converter(fl->rpm) ) * WHEEL_RADIUS;
    v_fr= Reduction_gearbox( RPM_converter(fr->rpm) ) * WHEEL_RADIUS;
    v_rl= - Reduction_gearbox( RPM_converter(rl->rpm) ) * WHEEL_RADIUS;
    v_rr= Reduction_gearbox( RPM_converter(rr->rpm) ) * WHEEL_RADIUS;

    Vl=(v_fl + v_rl)/2;
    Vr=(v_fr + v_rr)/2;

    //calculation of robot's velocity and robot's angular velocity
    Vk= (Vl + Vr)/2;
    Wz= (Vr - Vl)/APPARENT_BASELINE;

    //create TwistStamped message
    geometry_msgs::TwistStamped velocities;
    velocities.header.stamp=ros::Time::now();
    velocities.header.frame_id="velocities";
    velocities.twist.linear.x=Vk;
    velocities.twist.linear.y=0.0;
    velocities.twist.linear.z=0.0;
    velocities.twist.angular.x=0.0;
    velocities.twist.angular.y=0.0;
    velocities.twist.angular.z=Wz;

    ROS_INFO("Linear velocity: %f",Vk);
    ROS_INFO("Angular velocity: %f",Wz);

    //publish TwistStamped message
    velocities_pub.publish(velocities);

}


// test the callback function
void test_callback(const robotics_hw1::MotorSpeed::ConstPtr& fl,
                   const robotics_hw1::MotorSpeed::ConstPtr& fr,
                   const robotics_hw1::MotorSpeed::ConstPtr& rl,
                   const robotics_hw1::MotorSpeed::ConstPtr& rr){
    ROS_INFO("\n %f - %f \n %f - %f", fl->rpm, fr->rpm, rl->rpm, rr->rpm);
}

// calibrate the apparent baseline
void calibrate_app_baseline(const robotics_hw1::MotorSpeed::ConstPtr& fl,
                   const robotics_hw1::MotorSpeed::ConstPtr& fr,
                   const robotics_hw1::MotorSpeed::ConstPtr& rl,
                   const robotics_hw1::MotorSpeed::ConstPtr& rr,
                   const nav_msgs::Odometry::ConstPtr& scout_odom){
    double wz;
    double v_fl, v_fr, v_rl, v_rr;
    double  Vr,Vl;
    double app_baseline,delta;
    v_fl= - Reduction_gearbox( RPM_converter(fl->rpm) ) * WHEEL_RADIUS;
    v_fr= Reduction_gearbox( RPM_converter(fr->rpm) ) * WHEEL_RADIUS;
    v_rl= - Reduction_gearbox( RPM_converter(rl->rpm) ) * WHEEL_RADIUS;
    v_rr= Reduction_gearbox( RPM_converter(rr->rpm) ) * WHEEL_RADIUS;

    Vl=(v_fl + v_rl)/2;
    Vr=(v_fr + v_rr)/2;
    wz= scout_odom->twist.twist.angular.z;
    app_baseline= (Vr - Vl)/wz;
    if(app_baseline >-100 && app_baseline < 100){
        i+=1;
        delta=app_baseline-mean;
        mean+= delta/i;
    }
    ROS_INFO("Apparent baseline: %f", mean);
}

// calculate gear reduction
void calc_gear_reduction(const robotics_hw1::MotorSpeed::ConstPtr& fl,
                            const robotics_hw1::MotorSpeed::ConstPtr& fr,
                            const robotics_hw1::MotorSpeed::ConstPtr& rl,
                            const robotics_hw1::MotorSpeed::ConstPtr& rr,
                            const nav_msgs::Odometry::ConstPtr& scout_odom){
    double v_fl, v_fr, v_rl, v_rr,v_odom;
    double  Vr, Vl, V, gr;
    double delta;

    v_fl= -( RPM_converter(fl->rpm) ) * WHEEL_RADIUS;
    v_fr= ( RPM_converter(fr->rpm) ) * WHEEL_RADIUS;
    v_rl= -( RPM_converter(rl->rpm) ) * WHEEL_RADIUS;
    v_rr= ( RPM_converter(rr->rpm) ) * WHEEL_RADIUS;
    v_odom= scout_odom->twist.twist.linear.x;
    Vl=(v_fl + v_rl)/2;
    Vr=(v_fr + v_rr)/2;
    V= (Vl+Vr)/2;
    gr=v_odom/V;
    if(gr >0 && gr < 100){
        i+=1;
        delta=gr-mean;
        mean+= delta/i;
    }
    ROS_INFO("Gearbox reduction: %f", mean);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "velocities_node");
    ros::NodeHandle n;
    velocities_pub= n.advertise<geometry_msgs::TwistStamped>("velocities",1000);

    ROS_INFO("STATE: OK");

    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_fl(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_fr(n, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_rl(n, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> MS_rr(n, "motor_speed_rr", 1);
    //message_filters::Subscriber<nav_msgs::Odometry> scout_odom(n, "scout_odom", 1);
    //message_filters::Subscriber<geometry_msgs::PoseStamped> Pose(n, "gt_pose", 1);

    typedef message_filters::sync_policies::ApproximateTime <robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), MS_fl, MS_fr, MS_rl, MS_rr);
    sync.registerCallback(boost::bind(&velocities_Calculus, _1, _2, _3, _4));

    //message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed, nav_msgs::Odometry> sync(MS_fl, MS_fr, MS_rl, MS_rr, scout_odom, 10);
    //sync.registerCallback(boost::bind(&calibrate_app_baseline, _1, _2, _3, _4, _5));

    //message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed, nav_msgs::Odometry> sync(MS_fl, MS_fr, MS_rl, MS_rr, scout_odom, 10);
    //sync.registerCallback(boost::bind(&calc_gear_reduction, _1, _2, _3, _4, _5));

    ros::spin();

    return 0;
}