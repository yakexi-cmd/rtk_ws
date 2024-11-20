## 1.在ego-planner中更改膨胀半径，帮助机器人在狭窄区域仍能够识别到通道，而不是全部变成障碍物
  位于/home/beihang705/lidar_slam/rtk_ws/src/planner/plan_manage/launch/advanced_param.xml 中，需要更改<param name="grid_map/obstacles_inflation"     value="0.3" />   
## 2.滤除多余点云，在/home/beihang705/lidar_slam/rtk_ws/src/planner/plan_manage/launch/advanced_param.xml 中，
  更改<param name="grid_map/filter_height_min" value="0.05"/>  <!-- 基于车体坐标系高度做过滤-->
      <param name="grid_map/filter_height_max" value="0.5"/>  <!-- 基于车体坐标系高度做过滤-->
















### 注意事项
1.ego-planner中使用的坐标系是map坐标系，而dlio中使用的是odom
2.void clearAndInflateLocalMap();//清除和膨胀本地地图

inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  inline void inflatePoint2D(const Eigen::Vector2i& pt, int step, vector<Eigen::Vector2i>& pts);

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {

## 1.这个函数中通过设置step扩展膨胀区域，因此若想更改膨胀半径，需要改动step
inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
      int num=0;
      for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z) {
          pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }
  }

  inline void GridMap::inflatePoint2D(const Eigen::Vector2i& pt, int step, vector<Eigen::Vector2i>& pts) {
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y){
          pts[num++] = Eigen::Vector2i(pt(0) + x, pt(1) + y);
        }
  }

 