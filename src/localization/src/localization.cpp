#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "ImageSegment.h"
#include "Feature.h"
#include "Match.h"

ImageSegment image_segment;
Feature feature;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZI> laser;
	pcl::io::loadPCDFile("55_laser.pcd", laser);

	//点云分割
    image_segment.setPointCloud(laser);
    image_segment.segment();
    std::vector<pcl::PointCloud<pcl::PointXYZI> > segment_cloud = image_segment.getSegmentCloud();

    pcl::io::savePCDFile("hide.pcd", image_segment.getHideCloud());

    for(int i = 0; i < segment_cloud.size(); i ++)
    {
        std::stringstream ss;
        ss << i;
        ss << "_segment.pcd";
        pcl::io::savePCDFile(ss.str(), segment_cloud[i]);
    }

    //特征提取
    feature.setInputCloud(segment_cloud);
    feature.setGround(image_segment.getGround());
    feature.compute();

    //pcl::io::savePCDFile("border.pcd", feature.getBorder());
    pcl::io::savePCDFile("line.pcd", feature.getLines());

    return 0;
}
