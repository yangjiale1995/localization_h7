#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ImageSegment.h"
#include "Feature.h"
#include "Match.h"

ImageSegment image_segment;
Feature feature;
Match match;

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    double startTime = ros::Time::now().toSec();

    pcl::PointCloud<pcl::PointXYZI> laser;
    pcl::fromROSMsg(*msg, laser);

    //点云分割
    image_segment.setPointCloud(laser);
    image_segment.segment();

    //特征提取
    feature.setInputCloud(image_segment.getSegmentCloud());
    feature.setGround(image_segment.getGround());
    feature.compute();

    //特征匹配
    match.setPointCloud(feature.getFeaturePoints());
    match.setGroundMatrix(feature.getGroundMatrix());
    match.compute();

    Eigen::MatrixXf DoF = match.get6DoF();

    double endTime = ros::Time::now().toSec();
    std::cout << std::setprecision(15) << msg->header.stamp << '\t' << DoF(1) << '\t' << DoF(0) << '\t' << (endTime - startTime) << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, callback);

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    pcl::io::savePCDFile("map.pcd", match.getMap());
    return 0;
}
