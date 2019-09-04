#include <ros/ros.h>
#include <iostream>
#include <control_msgs/com2veh.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include "Common.h"

void callback(const control_msgs::com2veh::ConstPtr &msg)
{
    //std::cout << msg->vehicle_x << '\t' << msg->vehicle_y << std::endl;
}

void callback2(const novatel_gps_msgs::Inspva::ConstPtr &msg)
{
    //std::cout << std::setprecision(15) << msg->header.stamp << std::endl;
}

void callback3(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg)
{
    positionConf yang, jia;
    //yang.n_gps_seq = msg->header.seq;
    yang.lon = msg->lon;
    yang.lat = msg->lat;
    yang.height = msg->height;

    gps2xy(jia, yang);

    std::cout << std::setprecision(15) << msg->header.stamp << '\t' << jia.x <<  '\t' << jia.y <<  std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<control_msgs::com2veh>("/control/control_vcu", 1000, callback);
    ros::Subscriber sub2 = nh.subscribe<novatel_gps_msgs::Inspva>("/inspva", 1000, callback2);
    ros::Subscriber sub3 = nh.subscribe<novatel_gps_msgs::NovatelPosition>("/bestpos", 10000, callback3);

    ros::spin();
    return 0;
}
