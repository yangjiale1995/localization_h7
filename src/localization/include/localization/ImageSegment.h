#ifndef IMAGE_SEGMENT_H
#define IMAGE_SEGMENT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <queue>

const int N_SCANS = 16;     //垂直分辨率2度
const int HORIZON_SCANS = 1800;     //水平分辨率0.2度

//点云分簇类
class ImageSegment
{
public:
    //构造函数和析构函数
    ImageSegment();
    ~ImageSegment();

    pcl::PointCloud<pcl::PointXYZI> getFullCloud();

    //输入原始点云
    void setPointCloud(pcl::PointCloud<pcl::PointXYZI> laser);

    //得到分割后的点云簇
    std::vector<pcl::PointCloud<pcl::PointXYZI> > getSegmentCloud();
    

    //点云分割
    bool segment();

    pcl::PointCloud<pcl::PointXYZI> getGround();

private:

    //提取地面
    void detectGround();
    
    //分簇标记
    void labelComponent(int row, int col);
    
    //初始化
    void init();

    pcl::PointCloud<pcl::PointXYZI> full_cloud_;    //保存原始点云，按照行号排序

    Eigen::Matrix<float, N_SCANS, HORIZON_SCANS> range_mat_;    //点云距离矩阵
    Eigen::Matrix<int, N_SCANS, HORIZON_SCANS> ground_mat_;     //地面标志矩阵
    Eigen::Matrix<int, N_SCANS, HORIZON_SCANS> label_mat_;      //分簇标志矩阵
    
    int label_count_ = 1;       //每一簇的标志

    pcl::PointCloud<pcl::PointXYZI> ground_;        //地面点
    std::vector<pcl::PointCloud<pcl::PointXYZI> > segment_cloud_;   //分割点云
};


#endif
