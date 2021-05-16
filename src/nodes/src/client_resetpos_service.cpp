//
// Created by Domenico Parente
//
#include "ros/ros.h"
#include "nodes/ResetPos.h"
#include "cstring"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_resetpos_service");
    if (argc != 4)
    {
        ROS_INFO("Error! -> Usage: client_resetpos_service x y theta");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nodes::ResetPos>("resetpos_service");
    nodes::ResetPos srv;
    srv.request.x = atof(argv[1]);
    srv.request.y = atof(argv[2]);
    srv.request.theta = atof(argv[3]);
    if (client.call(srv))
    {
        ROS_INFO("Response: %s", srv.response.output.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call resetpos_service");
        return 1;
    }

    return 0;
}