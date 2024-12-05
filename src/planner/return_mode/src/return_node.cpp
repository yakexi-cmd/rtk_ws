#include "MotionGoalPub.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "return_mode");
    ros::NodeHandle nh;
    MotionGoalPub mgp(nh);
    ros::spin();
    
    return 0;
}
