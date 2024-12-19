// #include <ros/ros.h>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "ego_planner_node");
//   ros::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   // ros::MultiThreadedSpinner spinner(2);
//   // spinner.spin();
//   ros::spin();

//   return 0;
// }

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);
  ros::AsyncSpinner spinner_multi(1,&rebo_replan.multi_queue);
  spinner_multi.start();

  // ros::Duration(1.0).sleep();
  // ros::MultiThreadedSpinner spinner(1);
  ros::spin();

  return 0;
}
