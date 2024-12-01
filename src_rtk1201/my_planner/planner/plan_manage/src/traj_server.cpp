#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ego_planner/Bspline.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
//#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "time.h"

#define PI 3.1415926
#define yaw_error_max 90.0/180*PI
#define N 15

ros::Publisher vel_cmd_pub;

//quadrotor_msgs::PositionCommand cmd;
geometry_msgs::Twist cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool is_orientation_init = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_,time_s,time_e;
int traj_id_;

Eigen::Vector3d odom_pos_,odom_vel_;
Eigen::Quaterniond odom_orient_;

double roll,pitch,yaw;
geometry_msgs::PoseStamped pose_cur;
tf::Quaternion quat;
std_msgs::UInt8 is_adjust_pose;
std_msgs::UInt8 dir;

enum DIRECTION {POSITIVE=0,NEGATIVE=1};

// yaw control
double t_step;

std_msgs::UInt8 stop_command;

////time record
clock_t start_clock,end_clock;
double duration;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);


  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  //ROS_INFO("Receive b-spline trajectory!");

  receive_traj_ = true;

}

void poseCallback(geometry_msgs::PoseStampedConstPtr msg)
{
    pose_cur = *msg;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

void adjust_yaw_Callback(std_msgs::UInt8ConstPtr msg)
{
    is_adjust_pose = *msg;
}

void dirCallback(const std_msgs::UInt8ConstPtr& msg)
{
    dir = *msg;
}

void stopCallback(std_msgs::UInt8ConstPtr msg)
{
    stop_command = *msg;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(dir.data==NEGATIVE)
    {
        if(yaw>0)
        {
            yaw -= PI;
        }else if(yaw<0)
        {
            yaw += PI;
        }
    }
}

void cmdCallback(const ros::TimerEvent &e)
{
    /* no publishing before receive traj_ */
    if (stop_command.data==1)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        vel_cmd_pub.publish(cmd);
        return;
    }

    if (!receive_traj_)
        return;
    //ROS_WARN("Run here !");
    ros::Time time_s = ros::Time::now();
    double t_cur = (time_s - start_time_).toSec();

//    Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
//    Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
//    double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));

    static ros::Time time_last = ros::Time::now();

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        start_clock = clock();
        end_clock = clock();
        duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;
        //ROS_INFO("Control times : %f ms",duration);
    }
    else if (t_cur >= traj_duration_)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        vel_cmd_pub.publish(cmd);
        is_orientation_init=false;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_s;

    vel_cmd_pub.publish(cmd);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node("~");

  std::string cmd_topic,pose_topic;
  node.getParam("/ego_planner_node/fsm/pose_topic",pose_topic);
  node.getParam("/ego_planner_node/fsm/vel_topic",cmd_topic);


  ros::Subscriber bspline_sub = node.subscribe("/planning/bspline", 10, bsplineCallback);
  ros::Subscriber pose_sub = node.subscribe(pose_topic, 10, poseCallback);
  ros::Subscriber odom_sub = node.subscribe("/state_estimation", 10, odometryCallback);
  ros::Subscriber stop_sub = node.subscribe("/emergency_stop",10,stopCallback);
  ros::Subscriber adjust_yaw_sub = node.subscribe("/is_adjust_yaw",10,adjust_yaw_Callback);
  ros::Subscriber command_sub = node.subscribe("/direction",10,dirCallback);

  vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  stop_command.data = 0;
  dir.data = POSITIVE;
  t_step = 0.03;


  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.03), cmdCallback);

//  nh.param("traj_server/time_forward", time_forward_, -1.0);
//  last_yaw_ = 0.0;
//  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}