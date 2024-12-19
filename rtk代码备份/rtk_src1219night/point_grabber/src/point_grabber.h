#ifndef POINT_GRABBER_H
#define POINT_GRABBER_H
 
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
 
 
class PointGrabber : public rviz::Tool {
  Q_OBJECT
 
 public:
  PointGrabber();
  virtual int processMouseEvent(rviz::ViewportMouseEvent &event);
  virtual void activate();
  virtual void deactivate();
  virtual void onInitialize();
 
 private:
  void makeFlag(const Ogre::Vector3 &position);
 
  ros::Publisher pub_click_up;
  ros::NodeHandle nh_;
  geometry_msgs::Pose leftdown_ps;
  std::vector<Ogre::SceneNode *> flag_nodes_;
  Ogre::SceneNode *moving_flag_node_;
  std::string flag_resource_;
  nav_msgs::Path path_msg_;
  nav_msgs::Path shared_control_path_msg_;
  ros::Publisher path_pub_; 
  ros::Publisher shared_control_path_pub_; 
};
 
#endif  // SIMU_PARAM_PANEL_H