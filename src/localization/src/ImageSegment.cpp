#include "ImageSegment.h"
#include <iostream>
#define DEBUG 0

//构造函数
ImageSegment::ImageSegment()
{}

//析构函数
ImageSegment::~ImageSegment()
{}

//输入原始点云
void ImageSegment::setPointCloud(pcl::PointCloud<pcl::PointXYZI> laser)
{
#if DEBUG
    ROS_INFO_STREAM("set pointcloud function");
#endif
    
    init();     //第一步做初始化

    //构建range_mat_ 和 full_cloud_
    for(pcl::PointXYZI point : laser)
    {
        //计算行下标
        int row_ind = point.intensity;       //计算行下标
        if(row_ind < 0 || row_ind >= N_SCANS)
            continue;

        //计算列下标
        float horizon_angle = pcl::rad2deg(atan2(point.x, point.y)) + 180.0;
        if(horizon_angle > 360.0)
            continue;
        int column_ind = round(horizon_angle / 360.0 * HORIZON_SCANS);
        if(column_ind < 0 || column_ind >= HORIZON_SCANS)
            continue;

        //计算距离矩阵
        float range = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        range_mat_(row_ind, column_ind) = range;

        int index = column_ind + row_ind * HORIZON_SCANS;   //按行保存
        full_cloud_.points[index] = point;
    }
}


//检测地面
void ImageSegment::detectGround()
{
#if DEBUG
    ROS_INFO_STREAM("detect ground");
#endif

    //假设只有下8线能扫到地面
    for(int j = 0; j < HORIZON_SCANS; j ++)
    {
        for(int i = 0; i < N_SCANS / 2; i ++)
        {
            //相邻线同一水平角度的两个点下标
            int lower_ind = j + i * HORIZON_SCANS;
            int upper_ind = j + (i + 1) * HORIZON_SCANS;

            //intensity = -1 表示该角度没有扫描到点
            if(full_cloud_.points[lower_ind].intensity == -1 || full_cloud_.points[upper_ind].intensity == -1)
            {
                ground_mat_(i, j) = -1;
                continue;
            }

            //计算上下相邻两个点的三轴距离差
            float diff_x = full_cloud_.points[upper_ind].x - full_cloud_.points[lower_ind].x;
            float diff_y = full_cloud_.points[upper_ind].y - full_cloud_.points[lower_ind].y;
            float diff_z = full_cloud_.points[upper_ind].z - full_cloud_.points[lower_ind].z;
            //计算夹角，夹角过小表示地面点
            float angle = pcl::rad2deg(atan2(diff_z, sqrt(pow(diff_x, 2) + pow(diff_y, 2))));
            if(fabs(angle) <= 20)
            {
                ground_mat_(i, j) = 1;
                ground_mat_(i + 1, j) = 1;

                ground_.push_back(full_cloud_.points[lower_ind]);
                ground_.push_back(full_cloud_.points[upper_ind]);
            }
        }
    }

    for(int i = 0; i < N_SCANS; i ++)
    {
        for(int j = 0; j < HORIZON_SCANS; j ++)
        {
            //gound_mat_ = 1 表示地面点
            //range_mat_ = 0 表示不存在点
            //这些地方标记为-1
            if(ground_mat_(i, j) == 1 || range_mat_(i, j) == 0)
                label_mat_(i, j) = -1;
        }
    }

}


//向外扩展,广度优先遍历
void ImageSegment::labelComponent(int row, int col)
{
    bool line_count_flag[N_SCANS] = {false};    //同一簇的点分布在哪几个线
    std::queue<std::pair<int, int> > queue_ind;   //保存下标队列，先进先出
    std::vector<std::pair<int, int> > all_pushed_ind;       //保存成簇的下标
    pcl::PointCloud<pcl::PointXYZI> all_pushed_pointcloud;      //保存成簇的点

    //第一个点压进去
    queue_ind.push(std::pair<int, int>(row, col));  //广度优先遍历的先进先出队列
    all_pushed_ind.push_back(std::pair<int, int>(row, col));    //保存同一簇的所有下标
    all_pushed_pointcloud.push_back(full_cloud_.points[col + row * HORIZON_SCANS]);     //保存同一簇的所有点云

    //队列不空
    while(!queue_ind.empty())
    {
        //拿到第一个点下标
        std::pair<int, int> from_ind = queue_ind.front();
        queue_ind.pop();

        //添加标志
        label_mat_(from_ind.first, from_ind.second) = label_count_;

        for(int i = -1; i <= 1; i ++)   //上下两个点
        {
            for(int j = -1; j <= 1; j ++)   //左右两个点
            {
                //计算相邻点下标
                int neighborhood_ind_x = from_ind.first + i;
                int neighborhood_ind_y = (from_ind.second + j + HORIZON_SCANS) % HORIZON_SCANS;

                //上下左右相邻下标判断合法性
                if(neighborhood_ind_x < 0 || neighborhood_ind_x >= N_SCANS)
                    continue;
                //标志位为0表示该点不属于任何簇
                if(label_mat_(neighborhood_ind_x, neighborhood_ind_y) != 0)
                    continue;

                //点在full_cloud_中的下标计算
                int index_ind = from_ind.second + from_ind.first * HORIZON_SCANS;
                int index_neighborhood = neighborhood_ind_y + neighborhood_ind_x * HORIZON_SCANS;
                
                //距离差
                float diff_x = full_cloud_[index_ind].x - full_cloud_[index_neighborhood].x;
                float diff_y = full_cloud_[index_ind].y - full_cloud_[index_neighborhood].y;
                
                //相邻两个点距离足够近,是同一类点
                if(sqrt(pow(diff_x, 2) + pow(diff_y, 2)) < 1.0)
                {
                    queue_ind.push(std::pair<int, int>(neighborhood_ind_x, neighborhood_ind_y));
                    
                    label_mat_(neighborhood_ind_x, neighborhood_ind_y) = label_count_;

                    line_count_flag[neighborhood_ind_x] = true;

                    all_pushed_ind.push_back(std::pair<int, int>(neighborhood_ind_x, neighborhood_ind_y));
                    all_pushed_pointcloud.push_back(full_cloud_.points[index_neighborhood]);
                }
            }
        }
    }

    bool feasible_segment = false;  //判断是否是一簇点的标志
    if(all_pushed_pointcloud.points.size() >= 100)       //超过100个点为一簇
    {
        int line_count = 0;
        for(int i = 0; i < N_SCANS; i ++)
        {
            if(line_count_flag[i] == true)
                line_count ++;
        }
        if(line_count >= 3)
            feasible_segment = true;
    }
    else if(all_pushed_pointcloud.points.size() >= 8)     //没有超过100个点也可能是一簇，只要点分布在超过5个线也算一簇
    {
        int line_count = 0;
        for(int i = 0; i < N_SCANS; i ++)
        {
            if(line_count_flag[i] == true)
                line_count ++;
        }
        if(line_count >= 4)                 
            feasible_segment = true;
    }

    //如果构成一簇，那么label_count_标志位增1
    if(feasible_segment == true)
    {
        label_count_ ++;
        segment_cloud_.push_back(all_pushed_pointcloud);
    }
    else            //不构成一簇的点标记为999999
    {
        for(int i = 0; i < all_pushed_ind.size(); i ++)
        {
            label_mat_(all_pushed_ind[i].first, all_pushed_ind[i].second) = 999999;
        }
    }
}

//点云分割
bool ImageSegment::segment()
{
#if DEBUG
    ROS_INFO_STREAM("start segment");
#endif
   
    //输入点云为空
    if(full_cloud_.points.empty())
    {
#if DEBUG
        ROS_ERROR_STREAM("laser if empty");
#endif
        return false;
    }

    //地面检测
    detectGround();

    //点云分割
    for(int i = 0; i < N_SCANS; i ++)
    {
        for(int j = 0; j < HORIZON_SCANS; j ++)
        {
            if(label_mat_(i, j) == 0)       //表示该点没有被分类过
            {
                labelComponent(i, j);
            }
        }
    }

    return true;
}


//初始化
void ImageSegment::init()
{
    //初始化full_cloud_用NAN
    full_cloud_.resize(N_SCANS * HORIZON_SCANS);
    pcl::PointXYZI nan_point;
    nan_point.x = std::numeric_limits<float>::quiet_NaN();
    nan_point.y = std::numeric_limits<float>::quiet_NaN();
    nan_point.z = std::numeric_limits<float>::quiet_NaN();
    nan_point.intensity = -1;
    std::fill(full_cloud_.begin(), full_cloud_.end(), nan_point);

    //初始化矩阵
    range_mat_.setZero();
    ground_mat_.setZero();
    label_mat_.setZero();

    label_count_ = 1;

    ground_.clear();
    segment_cloud_.clear();

}


std::vector<pcl::PointCloud<pcl::PointXYZI> > ImageSegment::getSegmentCloud()
{
    return segment_cloud_;
}


pcl::PointCloud<pcl::PointXYZI> ImageSegment::getGround()
{
    return ground_;
}


pcl::PointCloud<pcl::PointXYZI> ImageSegment::getFullCloud()
{
    return full_cloud_;
}



