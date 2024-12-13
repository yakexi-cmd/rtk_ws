#include <atomic>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <queue>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <sys/vtimes.h>
#include <thread>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

// BOOST
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// DLIO
#include <nano_gicp/nano_gicp.h>
#include <direct_lidar_inertial_odometry/save_pcd.h>

// 将数值转换为具有指定经度的字符串
template <typename T> //定义模板函数，允许T是任意类型
std::string to_string_with_precision(const T a_value,const int n=6)
{
    std::ostringstream out;
    out.precision(n);
    out<<std::fixed << a_value;
    return out.str();
}

namespace dlio{
    enum class SensorType { OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN }; //枚举类定义不同传感器类型
    class OdomNode;
    class MapNode;
    struct Point{
        Point():data{0.f,0.f,0.f,1.f}{}
        PCL_ADD_POINT4D;//定义点结构的宏，确保point结构体在pcl中被识别为一个点类型
        float intensity; // intensity
        union{ //联合体，共享同一段内存，有一个变量被赋值后，其他变量的值不能确定
            std::uint32_t t; // (Ouster) time since beginning of scan in nanoseconds
            float time;      // (Velodyne) time since beginning of scan in seconds
            double timestamp;// (Hesai) absolute timestamp in seconds
        };
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(dlio::Point,
                                 (float,x,x)
                                 (float,y,y)
                                 (float,z,z)
                                 (float,intensity,intensity)
                                 (std::uint32_t,t,t) //虽然不同格式的时间都被注册（为了满足不同激光雷达的要求），但因为是联合体，只会有一个字段实际使用，其他字段可能保持默认值
                                 (float,time,time)
                                 (double,timestamp,timestamp))

typedef dlio::Point PointType;