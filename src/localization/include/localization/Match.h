#ifndef MATCH_H
#define MATCH_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/io/pcd_io.h>
#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <pcl/filters/voxel_grid.h>
#include "Common.h"


struct Point
{
    Point(double x, double y): x_(x), y_(y) {}
    template <typename T>
    bool operator()(const T* const pose, const T* const map_point, T* residual) const
    {
        Eigen::Matrix<T, 2, 2> R;
        R(0, 0) = ceres::cos(pose[2]);
        R(0, 1) = -ceres::sin(pose[2]);
        R(1, 0) = ceres::sin(pose[2]);
        R(1, 1) = ceres::cos(pose[2]);

        Eigen::Matrix<T, 2, 1> p;
        p(0) = T(x_);
        p(1) = T(y_);

        Eigen::Matrix<T, 2, 1> mp = R * p;

        residual[0] = T(map_point[0]) - (mp(0) + pose[0]);
        residual[1] = T(map_point[1]) - (mp(1) + pose[1]);
    }

    double x_;
    double y_;
};



//特征匹配类
class Match
{
public:
    //构造函数与析构函数
    Match();
    ~Match();

    //地面参数方程
    void setGroundMatrix(Eigen::Vector4f ground);

    //输入边缘点
    void setPointCloud(pcl::PointCloud<pcl::PointXYZI> laser);
    
    //计算
    void compute();

    //返回x,y,yaw
    Eigen::MatrixXf get6DoF();

    Eigen::Vector3f getVelocity();

    //返回地图点
    pcl::PointCloud<pcl::PointXYZI> getMap();

private:

    //计算roll,pitch,z
    void computeZRollPitch();

    //更新地图
    void updateMap();
    
    //处理遮挡
    void deleteHide(pcl::PointCloud<pcl::PointXYZI> laser);
    
    //相邻帧匹配
    void matchPreLaser();
    
    //地图点匹配
    void matchMap();

    //计算雅克比矩阵
    Eigen::MatrixXf computeJacobi(pcl::PointCloud<pcl::PointXYZI> laser, float yaw);

    pcl::PointCloud<pcl::PointXYZI> laser_;     //当前帧边缘点
    pcl::PointCloud<pcl::PointXYZI> pre_laser_; //上一帧边缘点
    
    pcl::PointCloud<pcl::PointXYZI> map_;   //地图点

    float delta_x_, delta_y_, delta_yaw_;
    float x_, y_, yaw_;
    float z_, roll_, pitch_;

    float pre_x_, pre_y_, pre_z_;
    float v_x_, v_y_, v_z_;

    Eigen::Vector4f ground_matrix_;     //地面参数方程

    bool first_laser_;      //第一帧边缘点标志
};

#endif
