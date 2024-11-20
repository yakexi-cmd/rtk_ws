//#include "utility.h"
#include <algorithm>
// #include <execution>   
#include <vector>   
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #include "robot_msg_705/ControlReply.h"

std::string output_type;
using namespace std;

// 前摆臂角度
double front_angle = 0;

static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};
static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    // uint8_t intensity;
    float intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                          (uint16_t, ring, ring)(double, timestamp, timestamp))

// velodyne的点云格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)

struct VelodynePointXYZIR {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)
)

ros::Subscriber subRobosensePC;
ros::Subscriber subCrawlerAngle;
ros::Subscriber subCamerapoints;
ros::Publisher pubRobosensePC;
ros::Publisher pubFilterRange;

const int front_min = -18;
const int front_max = 18;
const int rear_min = -18;
const int rear_max = 18;
double delta_angle = 5.0 / 180.0 * M_PI;
vector<double> delta_angle_cos;
vector<double> delta_angle_sin;

template<typename T>
bool has_nan(T point) {

    // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
    // pcl remove nan not work normally
    // ROS_ERROR("Containing nan point!");
    if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)) {
        return true;
    } else {
        return false;
    }
}

template<typename T>
void publish_points(T &new_pc, const sensor_msgs::PointCloud2 &old_msg) {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg.header;
    pc_new_msg.header.frame_id = "velodyne";
    pubRobosensePC.publish(pc_new_msg);
}

void rsHandler_XYZI(sensor_msgs::PointCloud2 pc_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIR>());
    pcl::fromROSMsg(pc_msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {
        if (has_nan(pc->points[point_id]))
            continue;

        VelodynePointXYZIR new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = pc->points[point_id].intensity;
        // remap ring id
        if (pc->height == 16) {
            new_point.ring = RING_ID_MAP_16[point_id / pc->width];
        } else if (pc->height == 128) {
            new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
        }
        pc_new->points.push_back(new_point);
    }

    publish_points(pc_new, pc_msg);
}


template<typename T_in_p, typename T_out_p>
void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

    // to new pointcloud
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        T_out_p new_point;
//        std::copy(pc->points[point_id].data, pc->points[point_id].data + 4, new_point.data);
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.intensity = pc_in->points[point_id].intensity;
//        new_point.ring = pc->points[point_id].ring;
//        // 计算相对于第一个点的相对时间
//        new_point.time = float(pc->points[point_id].timestamp - pc->points[0].timestamp);
        pc_out->points.push_back(new_point);
    }
}

template<typename T_in_p, typename T_out_p>
void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
    }
}

template<typename T_in_p, typename T_out_p>
void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
    }
}

// 解决默认的fromROSMsg的Failed to find match for field 'intensity'的问题
void add_intensity(const sensor_msgs::PointCloud2& pc_msg, pcl::PointCloud<RsPointXYZIRT>& pc_in) 
{
    for(int i = 0; i < pc_msg.width * pc_msg.height; ++i)
    {
        RsPointXYZIRT point;
        point.intensity = pc_msg.data[i * pc_msg.point_step + pc_msg.fields[0].offset + 12];
        pc_in.points[i].intensity = point.intensity;
    }
}

bool in_front(const VelodynePointXYZIRT& point)
{
    //转移到 x: 27cm y:10cm z:-50cm坐标系下
    float x = point.x - 0.27; //0.02
    float y = point.y - 0.1; //0.26
    float z = point.z + 0.5; //0.63
    // // 绕x轴选择i * 10度, 判断是否在多边形 x: -3.8 ~ 3.8 cm y: -5.5 ~ 5.5 cm z: -5.5 ~ 60 cm内 
    // for(int i = -18; i < 18; ++i)
    // {
    //     float y_new = y * delta_angle_cos[i - front_min] - z * delta_angle_sin[i - front_min];
    //     float z_new = y * delta_angle_sin[i - front_min] + z * delta_angle_cos[i - front_min];
    //     if(x > -0.38 && x < 0.38 && y_new > -0.055 && y_new < 0.055 && z_new > -0.055 && z_new < 0.6)
    //         return true;
    // }
    float radius = sqrt(pow(y,2) + pow(z, 2));
    if(x > -0.38 && x < 0.38 && radius < 0.6 && point.z > -0.63)
        return true;

    //转移到 x: -27cm y:10cm z:-50cm坐标系下
    x = point.x + 0.27;
    // for(int i = -18; i < 18; ++i)
    // {
    //     float y_new = y * delta_angle_cos[i - front_min] - z * delta_angle_sin[i - front_min];
    //     float z_new = y * delta_angle_sin[i - front_min] + z * delta_angle_cos[i - front_min];
    //     if(x > -0.38 && x < 0.38 && y_new > -0.055 && y_new < 0.055 && z_new > -0.055 && z_new < 0.6)
    //         return true;
    // }
    if(x > -0.38 && x < 0.38 && radius < 0.6 && point.z > -0.63)
        return true;

    return false;
}

bool in_rear(const VelodynePointXYZIRT& point)
{
    //转移到 x: 27cm y:60cm z:-50cm坐标系下
    float x = point.x - 0.27;
    float y = point.y - 0.6;
    float z = point.z + 0.5;
    float radius = sqrt(pow(y,2) + pow(z, 2));
    if(x > -0.38 && x < 0.38 && radius < 0.38 && point.z > -0.63)
        return true;
    // // 绕x轴选择i * 10度, 判断是否在多边形 x: -3.8 ~ 3.8 cm y: -5.5 ~ 5.5 cm z: -5.5 ~ 38.5 cm内 
    // for(int i = -18; i < 18; ++i)
    // {
    //     float y_new = y * delta_angle_cos[i - front_min] - z * delta_angle_sin[i - front_min];
    //     float z_new = y * delta_angle_sin[i - front_min] + z * delta_angle_cos[i - front_min];
    //     if(x > -0.38 && x < 0.38 && y_new > -0.055 && y_new < 0.055 && z_new > -0.055 && z_new < 0.38)
    //         return true;
    // }

    //转移到 x: -27cm y:60cm z:-50cm坐标系下
    x = point.x + 0.27;
    if(x > -0.38 && x < 0.38 && radius < 0.38 && point.z > -0.63)
        return true;
    // for(int i = -18; i < 18; ++i)
    // {
    //     float y_new = y * delta_angle_cos[i - front_min] - z * delta_angle_sin[i - front_min];
    //     float z_new = y * delta_angle_sin[i - front_min] + z * delta_angle_cos[i - front_min];
    //     if(x > -0.38 && x < 0.38 && y_new > -0.055 && y_new < 0.055 && z_new > -0.055 && z_new < 0.38)
    //         return true;
    // }
    return false;
}

bool in_bottom(const VelodynePointXYZIRT& point)
{
    // //长75 宽62 高23
    // //高-0.19 低 -0.47 半径范围 0.54
    // if(point.ring <= 16 && sqrt(pow(point.x,2) + pow(point.y, 2)) < 0.70 && point.z > -0.50 && point.z < -0.15)
    //     return true;
    // else
    //     return false;
    //计算yaw角
    float radius = sqrt(pow(point.x,2)+pow(point.y,2));
    float yaw = atan2(point.y, point.x)*180/M_PI;

    // 车体尺寸筛除
    if(point.x > - 0.75 && point.x < 0 && point.y > -0.41 && point.y < 0.41 && point.z > -0.47 && point.z < 0.20)
        return true;
    else if(abs(yaw) > 70 && abs(yaw) < 130 && (point.ring <= 7 || (point.ring == 10 && radius < 0.50))) 
        return true;
    else if(point.ring <= 10 && radius < 1.0 && abs(point.z) < 0.2)
        return true;
    else if(abs(yaw) > 130 && point.ring <= 5)
        return true;
    // 车体后半部分筛除
    else if(((point.ring <= 15 && radius <0.58 && 
            point.z > -0.47 && point.z < 0.20 && abs(yaw) >72)))
        return true;
    // 两根铝型材遮挡筛除
    // else if(radius < 0.35 && abs(point.z) <=0.05 && abs(yaw) > 85 && abs(yaw) < 95 || 
    //         radius < 0.45 && abs(point.z) <=0.05 && abs(yaw) > 85 && abs(yaw) < 95 && point.ring == 22)
    //     return true;
    // 无线模块和小盒子周围筛除
    else if(point.ring <= 13 && radius <1.48 && point.z > -0.42 && abs(yaw) > 160)
        return true;
    else
        return false;
}

bool in_radius(const VelodynePointXYZIRT& point)
{
    // 前9根线 半径范围
    
    if(point.ring <= 16 && sqrt(pow(point.x,2) + pow(point.y, 2)) < 1.15 && point.z > -0.60 && point.z < -0.50)
        return true;
    else
        return false;
}

bool in_polygon(const VelodynePointXYZIRT& point)
{
    // return in_front(point) || in_rear(point) || in_radius(point) || in_bottom(point);
    // return in_front(point) || in_rear(point);
    return (sqrt(pow(point.x,2) + pow(point.y, 2)) < 0.35 && point.z > -0.57) || in_bottom(point);
}


void point_cloud_filter(const pcl::PointCloud<VelodynePointXYZIRT>::Ptr& pc_in, pcl::PointCloud<VelodynePointXYZIRT>::Ptr& pc_out)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for(int i = 0; i < pc_in->points.size(); ++i)
    {
        if(in_polygon(pc_in->points[i]))
            continue;
        else{
            pc_out->points.push_back(pc_in->points[i]);
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::cout << "filter time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

    // std::for_each(std::execution::par, pc_in->begin(), pc_in->end(), [&](const auto& point)
    // {
    //     if (!in_polygon(point))
    //         pc_out->points.push_back(point);
    // });
}

void point_cloud_dynamic_filter(const pcl::PointCloud<VelodynePointXYZIRT>::Ptr& pc_in, pcl::PointCloud<VelodynePointXYZIRT>::Ptr& pc_out)
{
    double angle = front_angle;
    // y = 0.28, z = 0.63 圆盘半径10cm 长度62cm
    for(int i = 0; i < pc_in->points.size(); ++i)
    {
        // 转换到 x: 0cm y:28cm z:-63cm坐标系下 并绕-y轴旋转angle角度
        float x = pc_in->points[i].x - 0.0;
        float y = pc_in->points[i].y - 0.28;
        float z = pc_in->points[i].z + 0.73;
        float x_new = x * cos(angle) - z * sin(angle);
        float z_new = x * sin(angle) + z * cos(angle);
        float y_new = y;
        if(x_new > -0.10 && x_new < 0.10 && y_new > -0.05 && y_new < 0.05 && z_new > -0.10 && z_new < 0.52)
            continue;
        else
            pc_out->points.push_back(pc_in->points[i]);
    }
}

void rsHandler_XYZIRT(const sensor_msgs::PointCloud2 &pc_msg) {
    pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg(pc_msg, *pc_in);
    // add_intensity(pc_msg, *pc_in);

    if (output_type == "XYZIRT") {
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out_filtered(new pcl::PointCloud<VelodynePointXYZIRT>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_time<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        point_cloud_filter(pc_out, pc_out_filtered);
        // point_cloud_dynamic_filter(pc_out, pc_out_filtered);
        publish_points(pc_out_filtered, pc_msg);
        // publish_points(pc_out, pc_msg);
    } else if (output_type == "XYZIR") {
        pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIR>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_type == "XYZI") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>());
        handle_pc_msg<RsPointXYZIRT, pcl::PointXYZI>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    }
}

// void cameraPCLHandler(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*msg, *cloud);
//     std::cout << "test\n";
// }

// void subCrawlerAngleHandler(const robot_msg_705::ControlReply &pc_msg)
// {
//     // front_angle = pc_msg.motor_f.angle;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_repub_node");
    ros::NodeHandle nh;
    // 输出点云类型
    output_type = "XYZIRT";
    for(int i = front_min; i < front_max; i++)
    {
        delta_angle_cos.push_back(cos(i * 5.0 / 180.0 * M_PI));
        delta_angle_sin.push_back(sin(i * 5.0 / 180.0 * M_PI));
    }


    subRobosensePC = nh.subscribe("/rslidar_points", 1, rsHandler_XYZIRT);
    // subCamerapoints = nh.subscribe("/camera/depth_registered/points", 1, cameraPCLHandler);
    // subCrawlerAngle = nh.subscribe("/state_robot", 1, subCrawlerAngleHandler); //TODO: 话题名字
    pubRobosensePC = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
    pubFilterRange = nh.advertise<visualization_msgs::MarkerArray>("/filter_range", 1);

    ROS_INFO("Listening to /rslidar_points ......");
    ros::spin();
    return 0;
}
