//
// Created by Domenico Parente
//
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "custom_odometry/customOdometry.h"
#include "dynamic_reconfigure/server.h"
#include "nodes/integration_paramConfig.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Quaternion.h"
#include "nodes/Reset0.h"
#include "nodes/ResetPos.h"
#include "string.h"
#include "math.h"

// Initial position
double x=0.0;
double y=0.0;
double z=0.0;
double theta=0.0;
double t=0.0;

int odom_type;
tf::TransformBroadcaster *br;
ros::Publisher odom_pub;
ros::Publisher cu_odom_pub;
ros::Subscriber vel_sub;

void odomCalculus(const geometry_msgs::TwistStampedConstPtr& vel ){
    double Vk, w;
    double Ts;

    Vk= vel->twist.linear.x;
    w= vel->twist.angular.z;

    //computation Ts
    if (t==0.0)
    {
        Ts=0.0;
    }
    else
    {
        Ts= vel->header.stamp.toSec()- t;
    }
    t= vel->header.stamp.toSec();

    //computation Euler Integration
    if(!odom_type){
        x+= Vk * Ts * cos(theta);
        y+= Vk * Ts * sin(theta);
        theta+= w * Ts;
    }
    //computation Runge-Kutta Integration
    else{
        x+= Vk * Ts * cos(theta + (w * Ts)/2 );
        y+= Vk * Ts * sin(theta + (w * Ts)/2 );
        theta+= w * Ts;
    }

    ROS_INFO("x: %f - y: %f",x,y);

    // creation tf message
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //creation odometry message
    nav_msgs::Odometry odometry;
    odometry.header.stamp=ros::Time::now();
    odometry.header.frame_id="odom";

    odometry.pose.pose.position.x=x;
    odometry.pose.pose.position.y=y;
    odometry.pose.pose.position.z=0.0;
    odometry.pose.pose.orientation= odom_quat;

    odometry.twist.twist.linear.x=Vk;
    odometry.twist.twist.linear.y=0.0;
    odometry.twist.twist.linear.z=0.0;
    odometry.twist.twist.angular.x=0.0;
    odometry.twist.twist.angular.y=0.0;
    odometry.twist.twist.angular.z= w;

    //creation custom odom messages
    custom_odometry::customOdometry custom_odom;
    custom_odom.odometry=odometry;
    if(!odom_type){
        custom_odom.integration_type="euler";
    } else{
        custom_odom.integration_type="rk";
    }

    //publish messages
    odom_pub.publish(odometry);
    cu_odom_pub.publish(custom_odom);

    //send the transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

}

//set initial position
void set_initial_pos(double pos_x, double pos_y, double theta_angle){
    x= pos_x;
    y= pos_y;
    theta= theta_angle;
    ROS_INFO("Initial position set");
}

bool reset0(nodes::Reset0::Request &req,
            nodes::Reset0::Response &res){
    x= 0.0;
    y= 0.0;
    theta= 0.0;
    res.output="The position is set to (0,0,0)";
    ROS_INFO("Position reset to (0,0,0)");
    return true;
}

bool resetPos(nodes::ResetPos::Request  &req,
              nodes::ResetPos::Response &res){
    x= req.x;
    y= req.y;
    theta= req.theta;
    res.output="The position is set to the specified position";
    ROS_INFO("Position reset to (x,y,theta)");
    return true;
}

void dynamicCallback(nodes::integration_paramConfig &config, uint32_t level) {
    if(config.integration==0){
        odom_type=0;
        ROS_INFO("Changed mode to: Euler integration");
    }
    else {
        odom_type=1;
        ROS_INFO("Changed mode to: Runge-Kutta integration");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_node");
    br = new tf::TransformBroadcaster();
    ros::NodeHandle n;
    ROS_INFO("STATE: OK");
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    cu_odom_pub = n.advertise<custom_odometry::customOdometry>("custom_odom", 50);
    vel_sub = n.subscribe("velocities", 1000, odomCalculus);

    std::double_t pos_x;
    std::double_t pos_y;
    std::double_t theta_angle;
    // initialize the position of the robot through the parameters
    n.getParam("/position/x",pos_x);
    n.getParam("/position/y",pos_y);
    n.getParam("/position/theta",theta_angle);
    set_initial_pos(pos_x,pos_y,theta_angle);

    //set server for service to reset to (0,0,0) position
    ros::ServiceServer service_0 = n.advertiseService("reset0_service", reset0);
    //set server for service to reset to (x,y,theta) position
    ros::ServiceServer service_pos = n.advertiseService("resetpos_service", resetPos);
    //set server for dynamic reconfigure
    dynamic_reconfigure::Server<nodes::integration_paramConfig> server;
    dynamic_reconfigure::Server<nodes::integration_paramConfig>::CallbackType f;
    f = boost::bind(&dynamicCallback, _1,_2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
