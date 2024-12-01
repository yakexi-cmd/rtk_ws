/*
    使用均匀B样条生成和优化轨迹，得到一条满足速度，加速度，jerk约束的轨迹，应该属于后端轨迹优化部分
*/
#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;
namespace ego_planner
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
    class UniformBspline
    {
    private:
        // control points for B-spline with different dimensions.
        // Each row represents one single control point
        // The dimension is determined by column number
        // e.g. B-spline with N points in 3D space -> Nx3 matrix
        Eigen::MatrixXd control_points_; //存储b样条的控制点的矩阵

        int p_, n_, m_;     // p表示样条阶数，控制点数是n+1,m=n+p+1用于控制节点向量
        Eigen::VectorXd u_; // knots vector
        double interval_;   // 节点间隔或时间步长

        Eigen::MatrixXd getDerivativeControlPoints();

        double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio
    public:
        UniformBspline() {}
        UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
        ~UniformBspline();
        Eigen::MatrixXd get_control_points(void) { return control_points_; }
        //  B-spline初始化函数
        void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
        // get / set basic bspline info

        void setKnot(const Eigen::VectorXd &knot);//设置b样条的节点向量
        Eigen::VectorXd getKnot();//获得b样条的节点向量
        Eigen::MatrixXd getControlPoint();
        double getInterval();//获得b样条的时间间隔
        bool getTimeSpan(double &um, double &um_p);//获得b样条的时间跨度，um表示时间跨度的起始，um_p表示结束参数

        void getCloestT(const Eigen::Vector3d& pos, double& t_out, double& dis_out);//找到给定点pose在b样条上最近的点，输出对应的时间参数和距离
        // compute position / derivative

        Eigen::VectorXd evaluateDeBoor(const double &u);  //返回在u处的速度和加速度信息
        inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
        UniformBspline getDerivative();//获得b样条曲线的导数
        // 3D B-spline interpolation of points in point_set, with boundary vel&acc
        // constraints
        // input : (K+2) points with boundary vel/acc; ts
        // output: (K+6) control_pts
        static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                        const vector<Eigen::Vector3d> &start_end_derivative,
                                        Eigen::MatrixXd &ctrl_pts);
        /* check feasibility, adjust time */

        void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);//设置物理限制，包括速度，加速度和容忍度
        bool checkFeasibility(double &ratio, bool show = false);//检查b样条曲线是否满足物理限制
        void lengthenTime(const double &ratio);//如果不满足物理限制，根据给定的比例ratio调整时间参数，使曲线满足物理限制

        /* for performance evaluation */

        double getTimeSum();
        double getLength(const double &res = 0.01);
        double getJerk();//计算加速度的变化率
        void getMeanAndMaxVel(double &mean_v, double &max_v);
        void getMeanAndMaxAcc(double &mean_a, double &max_a);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                            
    };
}