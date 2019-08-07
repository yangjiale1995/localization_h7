#include "Common.h"

//按照角度排序
bool compareAngle(pcl::PointXYZI point1, pcl::PointXYZI point2)
{
    float angle1 = pcl::rad2deg(atan2(point1.y, point1.x));
    float angle2 = pcl::rad2deg(atan2(point2.y, point2.x));
    return angle1 < angle2;
}

//按照Ｘ轴排序
bool compareX(pcl::PointXYZI point1, pcl::PointXYZI point2)
{
    if((int)point1.intensity != (int)point2.intensity)
    {
        return (int)point1.intensity < (int)point2.intensity;
    }
    else
    {
        return (point1.x < point2.x);
    }
}

//按照Ｙ轴排序
bool compareY(pcl::PointXYZI point1, pcl::PointXYZI point2)
{
    if((int)point1.intensity != (int)point2.intensity)
    {
        return (int)point1.intensity < (int)point2.intensity;
    }
    else
    {
        return point1.y < point2.y;
    }
}

