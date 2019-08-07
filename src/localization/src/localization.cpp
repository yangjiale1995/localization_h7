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
Match match;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;

    for(int i = 0; i < 580; i ++)
    {
	//double start_time = ros::Time::now().toSec();

    	pcl::PointCloud<pcl::PointXYZI> laser;
	std::stringstream  ss;
	ss << i;
	ss << "_laser.pcd";
	pcl::io::loadPCDFile(ss.str(), laser);

	double start_time = ros::Time::now().toSec();
    	
	//点云分割
    	image_segment.setPointCloud(laser);
    	image_segment.segment();
    	std::vector<pcl::PointCloud<pcl::PointXYZI> > segment_cloud = image_segment.getSegmentCloud();

    	//特征提取
    	feature.setInputCloud(segment_cloud);
    	feature.setGround(image_segment.getGround());
    	feature.compute();

	ss.str("");
	ss << i;
	ss << "_line.pcd";
	pcl::io::savePCDFile(ss.str(), feature.getLines());

    	match.setPointCloud(feature.getFeaturePoints());
    	match.setGroundMatrix(feature.getGroundMatrix());
    	match.compute();

	Eigen::MatrixXf DoF = match.get6DoF();

	std::cout << DoF(0) << '\t' << DoF(1) << std::endl;
    	
	double end_time = ros::Time::now().toSec();

	//std::cout << "run time = " << end_time - start_time << std::endl;
    }

    return 0;
}
