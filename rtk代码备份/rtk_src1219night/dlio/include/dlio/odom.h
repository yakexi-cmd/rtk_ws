#include "dlio/dlio.h"

class dlio::OdomNode{
public:
    OdomNode(ros::NodeHandle node_handle);
    ~OdomNode();
    void start();
private:
    struct State; 
    struct ImuMeas;
    void getParams();
    void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
    void callbackImu(const sensor_msgs::Imu::ConstPtr& imu);
    void publishPose(const ros::TimerEvent& e);
    void publishImuPose(const ros::TimerEvent& e);
    void publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);
    void publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);
    void publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                       pcl::PointCloud<PointType>::ConstPtr> kf, ros::Time timestamp);
    void getScanFromROS(const sensor_msgs::PointCloud2ConstPtr& pc);//将ros点云数据转换为pcl类型，并预处理：首先去除无效点，检查点云数据字段确定雷达类型，然后将原始点云存入original_scan,点云时间戳存入scan_header_stamp
    void preprocessPoints();//用于逐点去畸变和统一预积分，如果imu数据到达，使用integrateImu函数，使用imu数据对雷达的运动进行积分?
    void deskewPointcloud();//对点云进行去畸变，将原始点云的点按照时间戳顺序复制到新的点云中deskewed_scan_，确保点云数据的时间对齐，便于后续的点云处理和匹配
    void deskewPointcloud2();//对点云进行去畸变，将原始点云的点按照时间戳顺序复制到新的点云中deskewed_scan_，确保点云数据的时间对齐，便于后续的点云处理和匹配
    void initializeInputTarget();//将第一帧作为关键帧，并在单独的线程中构建子地图和关键帧
    void setInputSource();//将新加入的点云帧作为输入源，传入GICP进行配准，用于点云匹配和位姿估计
    void initializeDLIO();//等待IMU数据的接收和校准dlio_initialized=true表示初始化完成，first_imu_received和imu_calibrated用于判断imu是否已经校准
    
    void getNextPose();//计算下一时刻的位姿，利用滤波或者优化的方法
    bool imuMeasFromTimeRange(double start_time, double end_time,//逆向迭代器，可以从后向前遍历，begin_imu_it和end_imu_it用于返回指定时间范围内的imu数据
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,//begin_imu_it指向end_time之后的第一个imu数据，end_imu_it指向start_time前的第一个imu数据
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps); //返回一个变换矩阵的向量，表示从初始状态到每个时间戳的坐标变换状态
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
    void propagateGICP();//GICP
    //积分，计算下一帧的位姿，并更新状态
    void propagateState();
    void updateState();
    // 设置自适应参数
    void setAdaptiveParams();
    void setKeyframeCloud();//设置关键帧点云
    // 计算环境相关的度量，如空间宽敞度和密度
    void computeMetrics();
    void computeSpaciousness();
    void computeDensity();
    sensor_msgs::Imu::Ptr transformImu(const sensor_msgs::Imu::ConstPtr& imu);//将imu转换到全局坐标系下
    void updateKeyframes();//用于更新关键帧和计算关键帧点云的凸包和凹包，有助于描述环境形状
    void computeConvexHull();
    void computeConcaveHull();
    void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);//用于确定哪些关键帧应该被包含在子地图中
    void buildSubmap(State vehicle_state);
    void buildKeyframesAndSubmap(State vehicle_state);
    void pauseSubmapBuildIfNeeded();
    void debug();

    ros::NodeHandle nh;
    ros::Timer publish_timer;
    // Subscribers
    ros::Subscriber lidar_sub;
    ros::Subscriber imu_sub;
    
    // Publishers
    ros::Publisher odom_pub;
    ros::Publisher imu_odom_pub;
    ros::Publisher pose_pub;
    ros::Publisher path_pub;
    ros::Publisher kf_pose_pub;
    ros::Publisher kf_cloud_pub;
    ros::Publisher deskewed_pub;

    // ROS Msgs:geometry_msgs::PoseStamped不包含机器人的速度加速度等信息，只包含位置和姿态及时间戳；nav_msgs::Odometry包含机器人运动的位置、方向、线速度和角速度信息
    nav_msgs::Odometry odom_ros;
    geometry_msgs::PoseStamped pose_ros;
    nav_msgs::Path path_ros;
    geometry_msgs::PoseArray kf_pose_ros;

    // / Flags：std::atomic多线程时只能有一个线程访问当前变量
    std::atomic<bool> dlio_initialized;
    std::atomic<bool> first_valid_scan;
    std::atomic<bool> first_imu_received;
    std::atomic<bool> imu_calibrated;
    std::atomic<bool> submap_hasChanged;
    std::atomic<bool> gicp_hasConverged;
    std::atomic<bool> deskew_status;
    std::atomic<int> deskew_size;

    // Threads
    std::thread publish_thread;
    std::thread publish_keyframe_thread;
    std::thread metrics_thread;
    std::thread debug_thread;

    // trajectory
    std::vector<std::pair<Eigen::Vector3f,Eigen::Quaternionf>> trajectory;
    double length_traversed;

    // Keyframes
    std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                            pcl::PointCloud<PointType>::ConstPtr>> keyframes;
    std::vector<ros::Time> keyframe_timestamps;
    std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals; //定义关键帧的法线信息
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations;//存放关键帧的变换矩阵信息
    std::mutex keyframes_mutex;

    // Sensor Type
    dlio::SensorType sensor;

    // Frames
    std::string odom_frame;
    std::string baselink_frame;
    std::string lidar_frame;
    std::string imu_frame;

    // Preprocessing滤波器：用于裁剪点云和对点云进行体素网格下采样
    pcl::CropBox<PointType> crop;
    pcl::VoxelGrid<PointType> voxel;

    // Point Clouds
    pcl::PointCloud<PointType>::ConstPtr original_scan;
    pcl::PointCloud<PointType>::ConstPtr deskewed_scan;
    pcl::PointCloud<PointType>::ConstPtr current_scan;
    // Keyframes
    pcl::PointCloud<PointType>::ConstPtr keyframe_cloud;
    int num_processed_keyframes;
    
    pcl::ConvexHull<PointType> convex_hull; //计算点云的凸包pcl类
    pcl::ConcaveHull<PointType> concave_hull;//计算点云凹包
    std::vector<int> keyframe_convex;
    std::vector<int> keyframe_concave;

    // Submap
    pcl::PointCloud<PointType>::ConstPtr submap_cloud;
    std::shared_ptr<const nano_gicp::CovarianceList> submap_normals;
    std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree;
    std::vector<int> submap_kf_idx_curr;
    std::vector<int> submap_kf_idx_prev;

    bool new_submap_is_ready;
    std::future<void> submap_future;
    std::condition_variable submap_build_cv;
    bool main_loop_running;
    std::mutex main_loop_running_mutex;

    // Timestamps
    ros::Time scan_header_stamp;
    double scan_stamp;
    double prev_scan_stamp;
    double scan_dt;
    std::vector<double> comp_times;
    std::vector<double> imu_rates;
    std::vector<double> lidar_rates;

    double first_scan_stamp;
    double elapsed_time;//存储某个特定事件到现在经过的时间

    // GICP
    nano_gicp::NanoGICP<PointType, PointType> gicp;
    nano_gicp::NanoGICP<PointType, PointType> gicp_temp;

    // Transformations
    Eigen::Matrix4f T, T_prior, T_corr;
    Eigen::Quaternionf q_final;

    Eigen::Vector3f origin;

    struct Extrinsics {
        struct SE3 {
        Eigen::Vector3f t;
        Eigen::Matrix3f R;
        };
        SE3 baselink2imu;
        SE3 baselink2lidar;
        Eigen::Matrix4f baselink2imu_T;
        Eigen::Matrix4f baselink2lidar_T;
    }; Extrinsics extrinsics;
    // IMU
    ros::Time imu_stamp;
    double first_imu_stamp;
    double prev_imu_stamp;
    double imu_dp, imu_dq_deg;

    struct ImuMeas {
        double stamp;
        double dt; // defined as the difference between the current and the previous measurement
        Eigen::Vector3f ang_vel;
        Eigen::Vector3f lin_accel;
    }; ImuMeas imu_meas; //imu中包含了线速度和角速度

    boost::circular_buffer<ImuMeas> imu_buffer;//固定大小的循环缓冲区，用于暂存imu数据，直到被整合到状态估计中
    std::mutex mtx_imu;
    std::condition_variable cv_imu_stamp;

    static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
        return (m1.stamp < m2.stamp);
    };

    // Geometric Observer
    struct Geo {
        bool first_opt_done;
        std::mutex mtx;
        double dp;
        double dq_deg;
        Eigen::Vector3f prev_p;
        Eigen::Quaternionf prev_q;
        Eigen::Vector3f prev_vel;
    }; Geo geo;

    // State Vector
    struct ImuBias {
        Eigen::Vector3f gyro;
        Eigen::Vector3f accel;
    };

    struct Frames {
        Eigen::Vector3f b;
        Eigen::Vector3f w;
    };

    struct Velocity {
        Frames lin;
        Frames ang;
    };

    struct State {
        Eigen::Vector3f p; // position in world frame
        Eigen::Quaternionf q; // orientation in world frame
        Velocity v;
        ImuBias b; // imu biases in body frame
    }; State state;

    struct Pose {
        Eigen::Vector3f p; // position in world frame
        Eigen::Quaternionf q; // orientation in world frame
    };
    Pose lidarPose;
    Pose imuPose;

    // Metrics
    struct Metrics {
        std::vector<float> spaciousness;
        std::vector<float> density;
    }; Metrics metrics;

    std::string cpu_type;
    std::vector<double> cpu_percents;
    clock_t lastCPU, lastSysCPU, lastUserCPU;
    int numProcessors;

    // Parameters
    std::string version_;
    int num_threads_;

    bool deskew_;

    double gravity_;

    bool time_offset_;

    bool adaptive_params_;

    double obs_submap_thresh_;
    double obs_keyframe_thresh_;
    double obs_keyframe_lag_;

    double keyframe_thresh_dist_;
    double keyframe_thresh_rot_;

    int submap_knn_;
    int submap_kcv_;
    int submap_kcc_;
    double submap_concave_alpha_;

    bool densemap_filtered_;
    bool wait_until_move_;

    double crop_size_;

    bool vf_use_;
    double vf_res_;

    bool imu_calibrate_;
    bool calibrate_gyro_;
    bool calibrate_accel_;
    bool gravity_align_;
    double imu_calib_time_;
    int imu_buffer_size_;
    Eigen::Matrix3f imu_accel_sm_;

    int gicp_min_num_points_;
    int gicp_k_correspondences_;
    double gicp_max_corr_dist_;
    int gicp_max_iter_;
    double gicp_transformation_ep_;
    double gicp_rotation_ep_;
    double gicp_init_lambda_factor_;

    double geo_Kp_;
    double geo_Kv_;
    double geo_Kq_;
    double geo_Kab_;
    double geo_Kgb_;
    double geo_abias_max_;
    double geo_gbias_max_;
};