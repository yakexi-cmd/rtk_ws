#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
// using namespace std;

namespace ego_planner
{
    void BsplineOptimizer::setParam(ros::NodeHandle &nh)
    {
        nh.param("optimization/lambda_smooth", lambda1_, -1.0);
        nh.param("optimization/lambda_collision", lambda2_, -1.0);
        nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
        nh.param("optimization/lambda_fitness", lambda4_, -1.0);

        nh.param("optimization/dist0", dist0_, -1.0);
        nh.param("optimization/max_vel", max_vel_, -1.0);
        nh.param("optimization/max_acc", max_acc_, -1.0);

        nh.param("optimization/order", order_, 3);
    }
    void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
    {
        this->grid_map_ = env;
    }
    void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
    {
        cps_.points = points;
    }