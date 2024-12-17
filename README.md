## 1.rtk
roslaunch nmea_navsat_driver nmea_serial_driver.launch

## 2.play rosbag
rosbag play 2024-11-25-10-35-54.bag

## subscriber topic:rtk_position
rostopic echo /rtk_position



## 12.14 更改
### 1.use lidar and imu (注意坐标系变换 在cfg/dlio.yaml!能够解决延迟问题) 
### 2.dlio运行时需要去除车体点云，防止机器人自身被识别为障碍物
### 3.膨胀半径要与 机器人对角线/2 一致

## 主要功能
### 1.dlio中存储激光slam，用于雷达的定位和建图，获得机器人相对于map坐标系的位置
### 2.planner中存储ego-planner,用于执行规划和避障
### 3.ws_cb是kuaixiang3的控制部分

## 运行命令 脚本
#!/bin/bash

gnome-terminal -- bash -c "sudo -S <<< "weiminlou705" chmod 777 /dev/ttyUSB0;"

gnome-terminal --window -e 'bash -c "source ~/disk/lidar_slam/ws_cb/devel/setup.bash; roscore; exec bash"' \
--tab -e 'bash -c "sleep 2;source ~/disk/lidar_slam/rtk_ws/devel/setup.bash; roslaunch direct_lidar_inertial_odometry lidar_slam.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source ~/disk/lidar_slam/rtk_ws/devel/setup.bash; roslaunch ego_planner run_in_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source ~/disk/lidar_slam/ws_cb/devel/setup.bash; roslaunch kuaixiang3_control kuaixiang3_control.launch; exec bash"' \


## 运行命令 终端
### 1.激光slam
source ~/disk/lidar_slam/rtk_ws/devel/setup.bash
roslaunch direct_lidar_inertial_odometry lidar_slam.launch

### 2.ego-planner
source ~/disk/lidar_slam/rtk_ws/devel/setup.bash
roslaunch ego_planner run_in_sim.launch

### 3.控制
source ~/disk/lidar_slam/ws_cb/devel/setup.bash
roslaunch kuaixiang3_control kuaixiang3_control.launch
