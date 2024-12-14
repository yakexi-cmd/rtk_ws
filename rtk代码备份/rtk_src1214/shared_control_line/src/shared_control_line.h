#ifndef SHARED_CONTROL_LINE_H
#define SHARED_CONTROL_LINE_H
 
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/tool.h>
#endif
 
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
 
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
 
 
class SharedControl : public rviz::Tool {
  Q_OBJECT
 
 public:
  SharedControl();
  virtual int processMouseEvent(rviz::ViewportMouseEvent &event);
  virtual void activate();
  virtual void deactivate();
  virtual void onInitialize();
 
 private:
  void makeFlag(const Ogre::Vector3 &position);
  void maker(const geometry_msgs::Pose &pose);
 
  ros::Publisher pub_click_up;
  ros::NodeHandle nh_;
  geometry_msgs::Pose leftdown_ps;
  geometry_msgs::PoseArray path_pose_array_;
  std::vector<Ogre::SceneNode *> flag_nodes_;
  Ogre::SceneNode *moving_flag_node_;
  std::string flag_resource_;
  ros::Publisher path_array_pub_; // 共享控制曲线屏幕点击的点的集合发布出去
  ros::Publisher path_marker_pub_; // 共享控制曲线屏幕点击的点的集合用marker显示
};
 
#endif  // SIMU_PARAM_PANEL_H