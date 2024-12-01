/*
    raycast是光线投射：将3d场景转换为2d，虚拟光线从相机焦点追踪在3d场景中对应的点和路径，计算图像中像素点对应的颜色和亮度，生成2d图像
    3d投射/仿射变换/射影变换：是一种几何变换，包含了仿射变换的所有操作（平移、旋转、缩放），模拟相机的透视效果，是一种数学上的映射
    主要应用于三维重建
*/
#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);//计算并返回沿某一方向，从s出发，到达下一个整数边界需要的距离

// 两个重载的射线投射函数，计算从start to end的射线与min-max定义的轴对齐边界框(AABB)的交点
// 函数使用了预分配的数组存储交点
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);
//动态存储交点
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

class RayCaster {
private:
    /* data */
    Eigen::Vector3d start_;
    Eigen::Vector3d end_;
    Eigen::Vector3d direction_;
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    int x_;
    int y_;
    int z_;
    int endX_;
    int endY_;
    int endZ_;
    double maxDist_;
    double dx_;
    double dy_;
    double dz_;
    int stepX_;//在每个轴向上步进的方向
    int stepY_;
    int stepZ_;
    double tMaxX_;//射线与AABB矩阵在x轴方向的两个平面(x.min,x.max)的交点
    double tMaxY_;
    double tMaxZ_;
    double tDeltaX_;//计算射线方向在xyz轴上的分量
    double tDeltaY_;
    double tDeltaZ_;
    double dist_;

    int step_num_;
    public:
    RayCaster(/* args */) {
    }
    ~RayCaster() {
    }
    bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool step(Eigen::Vector3d& ray_pt);//执行射线投射的单步操作，更新射线的位置并返回是否达到终点
};

#endif  // RAYCAST_H_