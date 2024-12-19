#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"

#include <geometry_msgs/PoseArray.h>


using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      ADJUST_POSE,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ego_planner::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_,last_state_;
    int continously_called_times_{0};

    /* motion data */
    geometry_msgs::Twist cmd_vel;
    std_msgs::UInt8 is_adjust_pose;
    double w_adjust;
    tf::Quaternion quat;
    bool is_pos_update;
    bool is_target_receive;
    bool is_multi_nav;//判断当前是否是多点导航
    int multi_goal_finish;//判断当前航点是否到达
    Eigen::Vector3d cur_goal;
    enum DIRECTION {POSITIVE=0,NEGATIVE=1};
    DIRECTION dir;
    bool have_multi_;
    
    std::vector<Eigen::Vector3d> multi_vec;
    std::deque<Eigen::Vector3d> multi_goal; //用于多点导航接收到的终点信息
    std::deque<geometry_msgs::PoseStamped> multi_goal_cmd;//发送给控制端的终点信息
    double roll,pitch,yaw,yaw_start,yaw_error_threshold,yaw_error;
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    geometry_msgs::PoseStamped  multi_end_pt_;
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    Eigen::Vector3d goal_last;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_,multi_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_,midpoint_sub_;
    ros::Subscriber multi_goal_sub;//用于订阅多点导航的目标点
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_,cmd_pub_,adjust_cmd_pub_,odom_adjust_pub_,dir_pub,stop_pub, stop_bool_pub,multi_goal_cmd_pub;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    bool planFromCurrentTrajNotByT();


    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void changeDirection();
    void checkYawError();
    double calculateYawError(double yaw_cur,double yaw_target);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    void getLocalTarget();
    void publishBspline();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void midpointCallback(const geometry_msgs::PoseStamped &msg);
    void multi_goal_callback(const geometry_msgs::PoseArray::ConstPtr &msg);
    void multiArriveCallback(const ros::TimerEvent &e);

    bool checkCollision();
    bool checkRotation(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }
    ros::CallbackQueue multi_queue; 

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
