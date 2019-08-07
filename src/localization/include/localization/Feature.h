#ifndef FEATURE_H
#define FEATURE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <algorithm>
#include "Common.h"

//特征提取类
class Feature
{
public:
    //构造函数和析构函数
    Feature();
    ~Feature();

    //输入原始点云
    void setInputCloud(std::vector<pcl::PointCloud<pcl::PointXYZI> > laser_segment);

    //特征提取程序运行函数
    void compute();

    //边缘点
    pcl::PointCloud<pcl::PointXYZI> getFeaturePoints();

    //边缘线
    pcl::PointCloud<pcl::PointXYZI> getLines();
    
    //轮廓
    pcl::PointCloud<pcl::PointXYZI> getBorder();

    //地面参数方程
    Eigen::Vector4f getGroundMatrix();

    void setGround(pcl::PointCloud<pcl::PointXYZI> ground);

private:

    //初始化
    void init();

    //计算地面参数方程
    void computeGroundMatrix();

    //提取平面函数
    void detectPlane(pcl::PointCloud<pcl::PointXYZI> laser);
    
    //提取边缘线函数
    void detectLine();
    
    //提取轮廓函数
    pcl::PointCloud<pcl::PointXYZI> detectBorder(pcl::PointCloud<pcl::PointXYZI> plane, Eigen::Vector3f normal);

    //轮廓
    pcl::PointCloud<pcl::PointXYZI> border_;

    //分割类和过滤器类
    pcl::ExtractIndices<pcl::PointXYZI> filter_;
    pcl::SACSegmentation<pcl::PointXYZI> segment_;

    //平面法线和边缘点
    pcl::PointCloud<pcl::PointXYZI> feature_points_;

    //保存分割后的点云簇
    std::vector<pcl::PointCloud<pcl::PointXYZI> > laser_segment_;

    //边缘线集合
    pcl::PointCloud<pcl::PointXYZI> lines_;

    //地面点云
    pcl::PointCloud<pcl::PointXYZI> ground_;

    //地面参数方程
    Eigen::Vector4f ground_matrix_;
};

#endif
