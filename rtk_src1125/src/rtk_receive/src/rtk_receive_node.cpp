#include "rtk_receive_node.hpp"

using namespace std;

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"rtk_receive_node");
  ros::NodeHandle nh("~");

  rtk_info rtk_;

  rtk_.receive_msg();

  ros::spin();

  return 0;
}