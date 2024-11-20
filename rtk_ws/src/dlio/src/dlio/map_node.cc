#include "dlio/map.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "dlio_map_node");
    ros::NodeHandle nh("~");
    dlio::MapNode node(nh);
    ros::AsyncSpinner spinner(0); //ros中用于异步处理回调的工具，允许节点处理回调函数的时候同时执行其他任务
    spinner.start();
    node.start();
    ros::waitForShutdown();
    return 0;
}