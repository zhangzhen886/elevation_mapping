# elevation-mapping

## 介绍
- 四足项目可行性区域检测，根据先验位姿，利用传感器数据稠密建图，为导航规划提供可行性区域的估计
- 具体采用了[四足机器人的ndt定位程序](https://gitee.com/csc105_slam_group/location)获取位姿信息，采用[Realsense D435i相机](https://gitee.com/sensors_and_external_devices_drive/realsense-ros)获取深度点云信息。采用detection节点对接定位信息传送给elevation_mapping节点（这里的elevation_mapping较github版本有修改），在elevation_mapping中根据每个栅格内点的方差大小确定可行与否

## 安装

### 开发环境

ubuntu16.04

ros Kinetic

### 依赖包

- [Grid Map](https://github.com/anybotics/grid_map) (grid map library for mobile robots)
- [kindr](http://github.com/anybotics/kindr) (kinematics and dynamics library for robotics)
- [kindr_ros](https://github.com/anybotics/kindr_ros) (ROS wrapper for kindr)
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing)
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library)


### 编译
```
catkin_make
```

## 节点参数含义说明
`./detection/launch/all.launch`包含了设置好的location、detection、elevation_mapping节点，具体节点参数的含义说明如下：

### 节点 detection

#### 订阅的话题名称
* **`/odom`** (nav_msgs::Odometry) 机器人的Odometry，这里是基于`/base_link`的frame
* **`/AreoCameraFront/depth/color/points`** (sensor_msgs::PointCloud2) 相机节点发布的深度点云（无颜色信息）

#### 发布的话题名称
* **`/odom_pose`** (geometry_msgs::PoseWithCovarianceStamped) 发布机器人pose节点，对接elevation_mapping节点需要的格式，基于`/base_link`的frame
* **`/camera_point`** (sensor_msgs::PointCloud2) 直通滤波后的相机点云数据
* **`/odom_pose_cameraframe`** (nav_msgs::Odometry) 发布相机pose节点，基于`cameraFrame`

#### 参数
* **`cameraFrame`** (string) 传感器（相机）frame的id
* **`FilterFieldName`** (string) 对相机点云进行直通滤波的通道名称
* **`setFilterLimitsUp`** (double) 直通滤波上界（单位：m）
* **`setFilterLimitsDown`** (double) 直通滤波下界（单位：m）

### 节点 elevation_mapping

#### 发布的话题名称
* **`/elevation_mapping/elevation_map`** (grid_map_msgs/GridMap) 以GridMap显示栅格地图（在rviz中调整
`Color Transformer`为`ColorLayer`，调整`Color Layer`为`height_color`得到可行区域的可视化显示）
* **`/elevation_map_raw_visualization/elevation_cloud`** (sensor_msgs/PointCloud2)以点云形式显示栅格地图

#### 订阅话题参数
* **`point_cloud_topic`** (string) 相机深度点云信息，格式为`sensor_msgs/PointCloud2`
* **`robot_pose_with_covariance_topic`** (string) 机器人位姿（不是相机位姿）,格式为`geometry_msgs/PoseWithCovarianceStamped`
* **`tf`** (string) 坐标变化关系，默认为`/tf`，不需要设置，格式为`tf/tfMessage`
* **`base_frame_id`** (string) 机器人frame的id
* **`map_frame_id`** (string) 地图frame的id
* **`sensor_frame_id`** (string) 传感器（相机）frame的id

#### 重要参数
* **`length_in_x` `length_in_y`** (double) 地图的长宽大小（单位：m）
* **`resolution`** (double) 地图的分辨率（单位：cm）
* **`threshold`** (double) 可行性区域不确定度阈值
* **`max_variance`** (double) 建立栅格的方差最大值
* **`robot_pose_cache_size`** (int) 位姿数据缓存的大小，越大则占用内存越高，根据位姿缓存匹配最近时间戳的传感器数据
* **`min_update_rate`** (double) 最小更新速率，一般设置为0.1即可
* **`fused_map_publishing_rate`** (double) 地图发布速率，对实时性影响较大，要考虑到pose和传感器数据的发布速率，一般设置为3.0

#### 可选参数
* **`sensor_processor/apply_voxelgrid_filter`** (bool) 进行体素滤波，提升实时性，但会导致准确性降低
* **`sensor_processor/voxelgrid_filter_size`** (double) 体素滤波方格大小（单位：cm）

* **`enable_visibility_cleanup`** (bool) 进行可视化清理，会影响实时性
* **`visibility_cleanup_rate`** (double) 可视化清理更新速率，会影响实时性
* **`scanning_duration`** (double) 可视化清理扫描速率，会影响实时性

* **`sensor_processor/type`** (string) 传感器类型，这里设置为`structured_light`
* **`sensor_processor/cutoff_max_depth`** (double) 传感器输入最大深度值（单位：m）
* **`sensor_processor/cutoff_min_depth`** (double) 传感器输入最小深度值（单位：m）
* **`sensor_processor/normal_factor_a` `sensor_processor/normal_factor_b` `sensor_processor/normal_factor_c` `sensor_processor/normal_factor_d` `sensor_processor/normal_factor_e` `sensor_processor/lateral_factor`** (double) 结构光传感器内参

## 效果图

<img alt="node data" src="image/result.png" width="700">
































