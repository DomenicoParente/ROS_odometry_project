//
// Created by Domenico Parente
//
#include "ros/ros.h"
#include "nodes/Reset0.h"
#include "cstring"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_reset0_service");
    if (argc != 1)
    {
        ROS_INFO("Error! -> Usage: client_reset0_service");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nodes::Reset0>("reset0_service");
    nodes::Reset0 srv;
    srv.request.input = 1;
    if (client.call(srv)){
        ROS_INFO("Response: %s", srv.response.output.c_str());
    }
    else{
        ROS_ERROR("Failed to call reset0_service");
        return 1;
    }

    return 0;
}