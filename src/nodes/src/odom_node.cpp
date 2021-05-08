//
// Created by Domenico Parente
//
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "custom_odometry/customOdometry.h"
#include "dynamic_reconfigure/server.h"
#include "nodes/parametersConfig.h"
#include <tf/transform_broadcaster.h>
#include "string.h"
#include "math.h"

// Initial position
double x=0.0;
double y=0.0;
double z=0.0;
double theta=0.0;
double t=0.0;

int odom_type;
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
        y+= Vk * Ts * cos(theta + (w * Ts)/2 );
        theta+= w * Ts;
    }

    //creation odometry message
    nav_msgs::Odometry odometry;
    odometry.header.stamp=ros::Time::now();
    odometry.header.frame_id="odometry";

    odometry.pose.pose.position.x=x;
    odometry.pose.pose.position.y=y;
    odometry.pose.pose.position.z=0.0;
    odometry.pose.pose.orientation= tf::createQuaternionMsgFromYaw(theta);;

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

    //tf publication
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
}


void dynamicCallback(nodes::parametersConfig &config, uint32_t level) {
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

    ros::NodeHandle n;
    ROS_INFO("STATE: OK");
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    cu_odom_pub = n.advertise<custom_odometry::customOdometry>("custom_odom", 50);
    vel_sub = n.subscribe("vel_subscriber", 1000, odomCalculus);

    //set server for dynamic reconfigure
    dynamic_reconfigure::Server<nodes::parametersConfig> server;
    dynamic_reconfigure::Server<nodes::parametersConfig>::CallbackType f;
    f = boost::bind(&dynamicCallback, _1,_2);
    server.setCallback(f);

    ros::spin();

    return 0;
}