#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <rviz/frame_manager.h>
#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
 
#include "point_grabber.h"
 
PointGrabber::PointGrabber() : moving_flag_node_(NULL) {
  pub_click_up = nh_.advertise<geometry_msgs::Pose>("/grab_a_point", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/multi_goal_path", 1);
  shared_control_path_pub_ = nh_.advertise<nav_msgs::Path>("/shared_control_path", 1);
}
 
void PointGrabber::onInitialize() {
  flag_resource_ = "package://point_grabber/media/flag.dae";
  if (rviz::loadMeshFromResource(flag_resource_).isNull()) {
    ROS_ERROR("PointGrabber: failed to load model resource '%s'.",flag_resource_.c_str());
    return;
  }
  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
  moving_flag_node_->attachObject(entity);
  moving_flag_node_->setVisible(false);
}
 
void PointGrabber::activate() {
  if (moving_flag_node_) moving_flag_node_->setVisible(true);
}
 
void PointGrabber::deactivate() {
  if (moving_flag_node_) moving_flag_node_->setVisible(false);
  for (unsigned i = 0; i < flag_nodes_.size(); i++)
    scene_manager_->destroySceneNode(flag_nodes_[i]);
  flag_nodes_.clear();
}
 
int PointGrabber::processMouseEvent(rviz::ViewportMouseEvent &event) {
  if (!moving_flag_node_) return Render;
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                        event.y, intersection)) {
    moving_flag_node_->setVisible(true);
    moving_flag_node_->setPosition(intersection);
    if (event.leftDown()) {
      // 添加当前点为路径点并更新路径
      shared_control_path_msg_.poses.clear(); // 初始化共享控制路径
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";  // 替换为合适的坐标系
      pose.pose.position.x = intersection.x;
      pose.pose.position.y = intersection.y;
      pose.pose.position.z = 0;
      pose.pose.orientation.w = 1.0;  // 默认朝向
      // 将点添加到路径中
      path_msg_.poses.push_back(pose);
      path_msg_.header = pose.header;  // 保持路径的 header 与点一致
      // 发布路径消息
      path_pub_.publish(path_msg_);

      makeFlag(intersection);
      leftdown_ps.position.x = intersection.x /*+ transform_.getOrigin().x()*/;  // event.x;
      leftdown_ps.position.y = intersection.y /*+ transform_.getOrigin().y()*/;  // event.y;
      leftdown_ps.position.z = 0;                           // 0;
      pub_click_up.publish(leftdown_ps);
      return Render;
    } else if (event.rightDown()) {
      // 清空路径并发布空路径
      shared_control_path_msg_ = path_msg_;
      shared_control_path_pub_.publish(shared_control_path_msg_);
      path_msg_.poses.clear();
      path_pub_.publish(path_msg_);
      return Render | Finished;
    }
  }
  return Render;
}
 
void PointGrabber::makeFlag(const Ogre::Vector3 &position) {
  Ogre::SceneNode *node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
  node->attachObject(entity);
  node->setVisible(false);
  node->setPosition(position);
  flag_nodes_.push_back(node);
}
 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PointGrabber,rviz::Tool)