#include "Feature.h"
#include <ros/ros.h>

#define DEBUG 0

//构造函数
Feature::Feature()
{
    //分割函数参数设置
    segment_.setOptimizeCoefficients(true);
    segment_.setDistanceThreshold(0.15);
    segment_.setMaxIterations(100);
    segment_.setProbability(0.8);

    //过滤器参数设置
    filter_.setNegative(true);
}

//析构函数
Feature::~Feature()
{}


//输入点云簇
void Feature::setInputCloud(std::vector<pcl::PointCloud<pcl::PointXYZI> > laser_segment)
{
#if DEBUG
    ROS_INFO_STREAM("set segmentation cloud");
#endif

    init();     //初始化
    laser_segment_ = laser_segment;     //点云簇
}

//初始化
void Feature::init()
{
    laser_segment_.clear();
    feature_points_.clear();
    border_.clear();
    lines_.clear();
    ground_.clear();
    ground_matrix_.setZero();
}

//计算特征
void Feature::compute()
{
#if DEBUG
    ROS_INFO_STREAM("start detect feature");
#endif

    computeGroundMatrix();      //计算地面参数方程
    
    //对每簇点云进行轮廓提取
    for(int i = 0; i < laser_segment_.size(); i ++)
    {
        detectPlane(laser_segment_[i]); 
    }

    //边缘线提取
    detectLine();

}

//计算地面参数方程
void Feature::computeGroundMatrix()
{
#if DEBUG
    ROS_INFO_STREAM("compute ground matrix");
#endif

    segment_.setModelType(pcl::SACMODEL_PLANE);
    segment_.setMethodType(pcl::SAC_RANSAC);
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    segment_.setInputCloud(ground_.makeShared());
    segment_.segment(*inliers, *coefficients);
    
    pcl::PointCloud<pcl::PointXYZI> yang;
    pcl::copyPointCloud(ground_, inliers->indices, yang);

    float angle = pcl::rad2deg(acos(Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]).dot(Eigen::Vector3f::UnitZ())));
    if(fabs(angle) < 40.0)
    {
        ground_matrix_(0) = coefficients->values[0];
        ground_matrix_(1) = coefficients->values[1];
        ground_matrix_(2) = coefficients->values[2];
        ground_matrix_(3) = coefficients->values[3];
    }
    else
    {
#if DEBUG
        ROS_ERROR_STREAM("detect ground failed");
#endif
        ground_matrix_(2) = 1.0;
    }
}



//提取平面
void Feature::detectPlane(pcl::PointCloud<pcl::PointXYZI> laser)
{
#if DEBUG
    ROS_INFO_STREAM("detect plane from segmented cloud");
#endif

    segment_.setModelType(pcl::SACMODEL_PLANE);
    segment_.setMethodType(pcl::SAC_RANSAC);

    //少于50个点可能为路灯立杆等标志
    if(laser.points.size() < 50)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setLeafSize(1.0, 1.0, 1.0);
        voxel_grid.setInputCloud(laser.makeShared());
        voxel_grid.filter(laser);

        border_ += laser;
    }

    //提取平面点
    while(laser.points.size() > 100)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        //输入点云进行平面分割
        segment_.setInputCloud(laser.makeShared());
        segment_.segment(*inliers, *coefficients);

        //少于100个点不认为是平面
        if(inliers->indices.size() < 100)
        {
            break;
        }

        //平面点
        pcl::PointCloud<pcl::PointXYZI> plane;
        pcl::copyPointCloud(laser, inliers->indices, plane);


        //过滤掉平面点
        filter_.setInputCloud(laser.makeShared());
        filter_.setIndices(inliers);
        filter_.filter(laser);

        //计算平面法线和z轴的夹角
        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        float delta = pcl::rad2deg(acos(normal.dot(Eigen::Vector3f::UnitZ())));
        
        //垂直于地面的平面为集装箱平面
        if(fabs(delta - 90.0) < 10.0)
        {
            pcl::PointCloud<pcl::PointXYZI> border = detectBorder(plane, normal);        //提取轮廓
            laser += border;
        }

    }
}


//提取轮廓
pcl::PointCloud<pcl::PointXYZI> Feature::detectBorder(pcl::PointCloud<pcl::PointXYZI> plane, Eigen::Vector3f normal)
{
#if DEBUG
    ROS_INFO("border.size() = %d", (int)border_.points.size());
    ROS_INFO_STREAM("detect border");
#endif

    Eigen::Vector2f n(normal(0), normal(1));
    Eigen::Vector2f x_axis(1, 0);
    
    //计算平面法线在XOY平面投影和X轴夹角
    float angle = pcl::rad2deg(acos(n.dot(x_axis) / n.norm()));
    //如果平面法线垂直于X轴，那么该平面与X轴平行，点云只需要按照X值从小到大排序
    if(fabs(angle - 90.0) < 40.0)
    {
        sort(plane.begin(), plane.end(), compareX);
    }
    //或者点云按照Y值从小到大排序
    else
    {
        sort(plane.begin(), plane.end(), compareY);
    }

    pcl::PointCloud<pcl::PointXYZI> border;
    //提取每一条线的最大值和最小值
    int scan_id = (int)plane.points[0].intensity;
    //border_.push_back(plane.points[0]);
    border.push_back(plane.points[0]);
    for(int i = 1; i < plane.points.size(); i ++)
    {
        if(scan_id != (int)plane.points[i].intensity)
        {
            border.push_back(plane.points[i - 1]);
            border.push_back(plane.points[i]);
            //border_.push_back(plane.points[i - 1]);
            //border_.push_back(plane.points[i]);
            scan_id = (int)plane.points[i].intensity;
        }
    }
    border.push_back(plane.points[plane.points.size() - 1]);
    //border_.push_back(plane.points[plane.points.size() - 1]);
    
    border_ += border;

    return border;
}

//返回轮廓点
pcl::PointCloud<pcl::PointXYZI> Feature::getBorder()
{
    return border_;
}

//从轮廓中提取边缘线点
void Feature::detectLine()
{
#if DEBUG
    ROS_INFO_STREAM("detect feature line");
#endif

    int points_thresh = 4;
    pcl::PointCloud<pcl::PointXYZI> border = border_;

    //设置参数
    segment_.setModelType(pcl::SACMODEL_LINE);
    segment_.setMethodType(pcl::SAC_RANSAC);
    segment_.setDistanceThreshold(0.1);

    while(border_.points.size() > points_thresh)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        segment_.setInputCloud(border.makeShared());
        segment_.segment(*inliers, *coefficients);

        //少于４个点
        if(inliers->indices.size() < points_thresh)
        {
            //边缘线少于２条，无法计算定位，需要降低提取标准继续提取特征点
            if(feature_points_.points.size() <= 2)
            {
                points_thresh --;
                if(points_thresh < 3)   //阈值过小
                {
                    break;
                    ROS_ERROR_STREAM("feature points less than 2");
                }
            }
            else
            {
                break;
            }
        }

        //提取到的线条
        pcl::PointCloud<pcl::PointXYZI> line;
        pcl::copyPointCloud(border, inliers->indices, line);

        //剔除掉线条点
        filter_.setInputCloud(border.makeShared());
        filter_.setIndices(inliers);
        filter_.filter(border);

        //计算直线方向向量和z轴夹角
        Eigen::Vector3f direction(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        float delta = pcl::rad2deg(acos(direction.dot(Eigen::Vector3f::UnitZ())));

        float yang = pcl::rad2deg(atan2(coefficients->values[1], coefficients->values[0]));

        //夹角足够小就保存下来(阈值可调)
        if(fabs(delta) < 10.0)
        {
            lines_ += line;

            float t = -coefficients->values[2] / coefficients->values[5];
            pcl::PointXYZI point;
            point.x = coefficients->values[1] + t * coefficients->values[4];
            point.y = coefficients->values[0] + t * coefficients->values[3];
            point.z = 0;
            feature_points_.push_back(point);
        }
    }

#if DEBUG
    ROS_INFO("lines: %d", (int)feature_points_.points.size());
#endif
}

//返回边缘线
pcl::PointCloud<pcl::PointXYZI> Feature::getLines()
{
    return lines_;
}

//返回边缘点
pcl::PointCloud<pcl::PointXYZI> Feature::getFeaturePoints()
{
    return feature_points_;
}


void Feature::setGround(pcl::PointCloud<pcl::PointXYZI> ground)
{
    ground_ = ground;
}


Eigen::Vector4f Feature::getGroundMatrix()
{
    return ground_matrix_;
}
