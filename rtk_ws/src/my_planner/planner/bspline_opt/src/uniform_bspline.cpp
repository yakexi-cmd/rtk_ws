#include "bspline_opt/uniform_bspline.h"
#include <ros/ros.h>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }
  UniformBspline::~UniformBspline() {}
  