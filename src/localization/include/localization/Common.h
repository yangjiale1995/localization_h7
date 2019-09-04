#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>

//按照角度排序
bool compareAngle(pcl::PointXYZI point1, pcl::PointXYZI point2);

//按照Ｘ轴排序
bool compareX(pcl::PointXYZI point1, pcl::PointXYZI point2);

//按照Ｙ轴排序
bool compareY(pcl::PointXYZI point1, pcl::PointXYZI point2);

//按照size大小需要
bool compareSize(pcl::PointCloud<pcl::PointXYZI> laser1, pcl::PointCloud<pcl::PointXYZI> laser2);

#endif
