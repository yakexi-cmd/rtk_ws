#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

using namespace Eigen;

enum MotionState
{
    SearchMode,
    ReturnMode
};

class MotionGoalPub
{
private:

    MotionState motion_state_ = SearchMode; // 默认为搜索模式

    Vector3d last_pos_ = {0.0, 0.0, 0.0};
    Vector3d curr_pos_ = {0.0, 0.0, 0.0};
    Vector3d target_pt_ = {0.0, 0.0, 0.0};

    pcl::PointCloud<PointT>::Ptr keyPoints_;
    pcl::KdTreeFLANN<PointT> kdtree_;
    int K = 5;
    int indices_;
    Vector3d lastKeyPoint_;
    Vector3d normalAtObs_;
    Vector3d pointAtObs_;

    // ros相关
    ros::NodeHandle nh_;
    ros::Subscriber returnMode_sub_;
    ros::Subscriber pose_sub_;
    ros::Timer goal_pub_timer_;
    ros::Publisher goal_pub_;
    ros::Publisher vis_keypoints_pub_;
    visualization_msgs::MarkerArray marker_array_msgs_;


    // debug文件
    // std::ofstream outputFile_;
    std::ofstream keyPointDebug_;
public:
    MotionGoalPub(ros::NodeHandle& nh);
    ~MotionGoalPub();

    void odometryCallback(const nav_msgs::OdometryConstPtr msg);
    void returnModeCallback(const std_msgs::Bool::Ptr& msg);
    void goalPubTimerCallback(const ros::TimerEvent& event);

    /** 寻找kdTree中最近的点
     * @param pt_in 搜索点
     * @param pt_out 找到的最近点
     * @param ind0 最近点在点云中的index
    */
    void searchNearestPoint(const Vector3d& pt_in, Vector3d& pt_out, int& ind0);

    void visKeyPoints();
};

