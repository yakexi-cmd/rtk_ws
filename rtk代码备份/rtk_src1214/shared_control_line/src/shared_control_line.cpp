#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <rviz/frame_manager.h>
#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
 
#include "shared_control_line.h"
 
SharedControl::SharedControl() : moving_flag_node_(NULL) {
  pub_click_up = nh_.advertise<geometry_msgs::Pose>("/grab_a_point", 1);
  path_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path_array", 1);
  path_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_ponits_marker", 1);
}
 
void SharedControl::onInitialize() {
  flag_resource_ = "package://shared_control_line/media/flag.dae";
  if (rviz::loadMeshFromResource(flag_resource_).isNull()) {
    ROS_ERROR("SharedControl: failed to load model resource '%s'.",flag_resource_.c_str());
    return;
  }
  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
  moving_flag_node_->attachObject(entity);
  moving_flag_node_->setVisible(false);
}
 
void SharedControl::activate() {
  if (moving_flag_node_) moving_flag_node_->setVisible(true);
}
 
void SharedControl::deactivate() {
  if (moving_flag_node_) moving_flag_node_->setVisible(false);
  for (unsigned i = 0; i < flag_nodes_.size(); i++)
    scene_manager_->destroySceneNode(flag_nodes_[i]);
  flag_nodes_.clear();
}
 
int SharedControl::processMouseEvent(rviz::ViewportMouseEvent &event) {
  if (!moving_flag_node_) return Render;
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                        event.y, intersection)) {
    moving_flag_node_->setVisible(true);
    moving_flag_node_->setPosition(intersection);
    if (event.leftDown()) {
      makeFlag(intersection);
      leftdown_ps.position.x = intersection.x /*+ transform_.getOrigin().x()*/;  // event.x;
      leftdown_ps.position.y = intersection.y /*+ transform_.getOrigin().y()*/;  // event.y;
      leftdown_ps.position.z = 0;                           // 0;
      pub_click_up.publish(leftdown_ps);
      // maker(leftdown_ps);
      path_pose_array_.poses.push_back(leftdown_ps);

      return Render;
    } else if (event.rightDown()) {
      // std::cout << path_pose_array_.poses.size() << std::endl;
      path_array_pub_.publish(path_pose_array_);
      // 初始化
      path_pose_array_.poses.clear();
      // visualization_msgs::Marker marker_delete;
      // marker_delete.action = visualization_msgs::Marker::DELETEALL;
      // path_marker_pub_.publish(marker_delete);
      return Render | Finished;
    }
  }
  return Render;
}
void SharedControl::maker(const geometry_msgs::Pose &pose) {
      // visualization_msgs::Marker marker;
      // marker.type = visualization_msgs::Marker::SPHERE;
      // marker.id = path_pose_array_.poses.size() - 1;  // 或者其他方法设置 ID
      // marker.pose = pose;
      // marker.scale.x = 0.5;  // 半径为 1.0 的球体
      // marker.scale.y = 0.5;
      // marker.scale.z = 0.5;
      // marker.color.r = 1.0;  // 红色
      // marker.color.g = 0.0;
      // marker.color.b = 0.0;
      // marker.color.a = 1.0;  // 不透明
      // marker.header.frame_id = "map";  // 或者其他适合的坐标系
      // marker.header.stamp = ros::Time::now();  // 时间戳
      // path_marker_pub_.publish(marker);
}

void SharedControl::makeFlag(const Ogre::Vector3 &position) {
  Ogre::SceneNode *node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *entity = scene_manager_->createEntity(flag_resource_);
  node->attachObject(entity);
  node->setVisible(false);
  node->setPosition(position);
  flag_nodes_.push_back(node);
}
 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SharedControl,rviz::Tool)