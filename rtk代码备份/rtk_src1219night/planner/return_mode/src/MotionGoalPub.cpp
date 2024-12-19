#include "MotionGoalPub.h"

MotionGoalPub::MotionGoalPub(ros::NodeHandle& nh)
{
    pose_sub_               = nh_.subscribe
        ("/Odometry",    100, &MotionGoalPub::odometryCallback,   this);
    returnMode_sub_         = nh_.subscribe
        ("/return_mode", 100, &MotionGoalPub::returnModeCallback, this);
    goal_pub_timer_         = nh_.createTimer
        (ros::Duration(0.1), &MotionGoalPub::goalPubTimerCallback, this);
    goal_pub_               = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 100);
    vis_keypoints_pub_      = nh_.advertise<visualization_msgs::MarkerArray>("/keypoints", 100);

    std::string package_path = ros::package::getPath("return_mode");
    keyPointDebug_.open(package_path + "/log/keypoint_log.txt", std::ios::out | std::ios::trunc);

    keyPoints_.reset(new PointCloud);
    PointT s_pt(curr_pos_(0), curr_pos_(1), curr_pos_(2));
    keyPoints_->points.push_back(s_pt);
    std::cout << "keyPoints size: " << keyPoints_->points.size() << "\n";
    kdtree_.setInputCloud(keyPoints_);
}

MotionGoalPub::~MotionGoalPub()
{
}


void MotionGoalPub::odometryCallback(const nav_msgs::OdometryConstPtr msg)
{
    curr_pos_ = Vector3d(msg->pose.pose.position.x, 
                         msg->pose.pose.position.y, 
                         msg->pose.pose.position.z);
    double dis_xy = Vector2d(curr_pos_(0) - last_pos_(0), 
                             curr_pos_(1) - last_pos_(1)).norm();
    
    if(motion_state_ == SearchMode)
    {
        // if(last_pos_.norm() == 0 || dis_xy > 2.0)
        // {
        //     keyPointDebug_ << "处于搜索模式，插入关键节点： " << "\n";
        //     keyPointDebug_ << "三维坐标为： x: " << curr_pos_.x() 
        //                    << "  y: " << curr_pos_.y() 
        //                    << "  z: " << curr_pos_.z() << "\n";
        //     PointT point(curr_pos_(0), curr_pos_(1), curr_pos_(2));
        //     keyPoints_->points.push_back(point);
        //     last_pos_ = curr_pos_;
        // }
        if(keyPoints_->points.size() != 0)  kdtree_.setInputCloud(keyPoints_);
        else return;
        Vector3d nearest_pos;
        int ind;
        searchNearestPoint(curr_pos_, nearest_pos, ind);
        if((curr_pos_ - nearest_pos).norm() > 4.0)
        {
            keyPointDebug_ << "处于搜索模式，插入关键节点： " << "\n";
            keyPointDebug_ << "三维坐标为： x: " << curr_pos_.x() 
                            << "  y: " << curr_pos_.y() 
                            << "  z: " << curr_pos_.z() << "\n";
            PointT point(curr_pos_(0), curr_pos_(1), curr_pos_(2));
            keyPoints_->points.push_back(point);
            last_pos_ = curr_pos_;
        }
    }
}



void MotionGoalPub::returnModeCallback(const std_msgs::Bool::Ptr& msg)
{
  if(msg->data) motion_state_ = ReturnMode;
  else motion_state_ = SearchMode;
}

void MotionGoalPub::goalPubTimerCallback(const ros::TimerEvent& event)
{
    visKeyPoints();
    if(keyPoints_->points.size() != 0)  kdtree_.setInputCloud(keyPoints_);
    else return;
    if(motion_state_ == ReturnMode)
    {
        if(curr_pos_.norm() < 0.1)
        {
            keyPointDebug_ << "到达起始点，重置为SearchMode" << "\n";
            motion_state_ = SearchMode;
            return;
        }

        keyPointDebug_ << "处于返航模式： 提取关键点： " << "\n";
        Vector3d nearest_pos;
        int ind;
        searchNearestPoint(curr_pos_, nearest_pos, ind);
        keyPointDebug_ << "提取到的关键点坐标为： x: " << nearest_pos(0) << "  y: " << nearest_pos(1) << "  z: " << nearest_pos(2) << "\n"; 
        if((nearest_pos - target_pt_).norm() > 0.5)
        {
            target_pt_ = nearest_pos;
            indices_ = ind;
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "map";
            goal_msg.pose.position.x = target_pt_(0);
            goal_msg.pose.position.y = target_pt_(1);
            goal_msg.pose.position.z = target_pt_(2);
            goal_msg.pose.orientation.w = 1.0;
            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = 0.0;
            goal_pub_.publish(goal_msg);
        }
        if((target_pt_ - curr_pos_).norm() < 0.5 && target_pt_.norm() > 0.05)
        {
            keyPoints_->points.erase(keyPoints_->points.begin() + indices_);
            keyPointDebug_ << "在kdtree中清除了该关键点" << "\n";
        }
    }
}

void MotionGoalPub::searchNearestPoint(const Vector3d& pt_in, Vector3d& pt_out, int& ind0)
{
    PointT search_point(pt_in(0), pt_in(1), pt_in(2));
    std::vector<int> indices(K);
    std::vector<float> distance(K);
    kdtree_.nearestKSearch(search_point, K, indices, distance);
    PointT nearest_point = keyPoints_->points[indices[0]];
    pt_out(0) = nearest_point.x;
    pt_out(1) = nearest_point.y;
    pt_out(2) = nearest_point.z;
    ind0 = indices[0];
}

void MotionGoalPub::visKeyPoints()
{
    if(marker_array_msgs_.markers.size() != 0)
    {
        for(int i = 0; i < keyPoints_->points.size(); i++)
        {
            marker_array_msgs_.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        marker_array_msgs_.markers.clear();
    }
    vis_keypoints_pub_.publish(marker_array_msgs_);
    for(size_t i = 0; i < keyPoints_->points.size(); i++)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "points";
        m.id = i;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = keyPoints_->points[i].x;
        m.pose.position.y = keyPoints_->points[i].y;
        m.pose.position.z = keyPoints_->points[i].z;
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
        marker_array_msgs_.markers.push_back(m);
    }
    vis_keypoints_pub_.publish(marker_array_msgs_);
}