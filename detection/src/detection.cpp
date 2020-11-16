#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <angles/angles.h>
#include <cmath>
#include <vector>
#include <iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/core/core.hpp>

using namespace std;
ros::Publisher g_pubOdomPose;
ros::Publisher g_pubOdomPose_Cameraframe;
ros::Subscriber g_subOdom;
ros::Publisher g_pubRScameraPoint;
ros::Subscriber g_subRScameraPoint;
ros::Publisher elevation_map_hz_pub;
tf::StampedTransform tf_lidar2base;
tf::StampedTransform tf_camera2lidar;
typedef pcl::PointXYZ cam_PointT;

// 节点参数
string cameraFrame = "/AreoCameraFront_depth_optical_frame";
string lidarFrame = "/velodyne";
string baseFrame = "/base_link";
string odomFrame = "/odom";
string camera_point_topic = "/AreoCameraFront/depth/color/points";
//double RPY[3];
string FilterFieldName;
double setFilterLimitsUp;
double setFilterLimitsDown;
bool use_static_tf;
int elevation_wait_time;

void publichTF_Rscamera(tf::TransformBroadcaster &Rscamera_broadcaster)
{
  //建立相机外参
  Eigen::Matrix3d lidar2cam_rotation;
  lidar2cam_rotation << -0.0036999,   -0.999962, 0.00784481,
                         -0.278016, -0.00650698,  -0.960554,
                          0.960569, -0.00573494,  -0.277981;
  Eigen::Isometry3d lidar2cam(lidar2cam_rotation);//可以是旋转矩阵、四元数、旋转向量
  lidar2cam.pretranslate(Eigen::Vector3d(0.0119567, -0.111237, -0.0959151));//添加平移向量
  Eigen::Matrix3d cam2lidar_rotation;
//  cam2lidar_rotation << -0.0036999,   -0.278016,    0.960569,
//                         -0.999962, -0.00650698, -0.00573494,
//                        0.00784481,   -0.960554,   -0.277981;
//  Eigen::Isometry3d cam2lidar(cam2lidar_rotation);//可以是旋转矩阵、四元数、旋转向量
//  cam2lidar.pretranslate(Eigen::Vector3d(0.0612516, 0.0106824, -0.133606));//添加平移向量
  cam2lidar_rotation << 0.0134529,   -0.366897,   0.930164,
                        -0.999906, -0.00239042,  0.0135187,
                      -0.00273649,   -0.930259,  -0.366895;
  Eigen::Isometry3d cam2lidar(cam2lidar_rotation);//可以是旋转矩阵、四元数、旋转向量
  cam2lidar.pretranslate(Eigen::Vector3d(0.0725806, 0.0172741, -0.014909));//添加平移向量

  //发布tf
  tf::Transform transform;
  tf::transformEigenToTF(cam2lidar, transform);//转化为tf变换的形式
//  transform.setOrigin( tf::Vector3(0.05, 0.05, -0.1) );//设置新坐标系相对位置关系

//  tf::Quaternion q;// 根据欧拉角设置旋转关系
//  double pi = 3.1415926;
//  q.setRPY(RPY[0] * pi / 180, RPY[1] * pi / 180, RPY[2] * pi / 180);//用弧度表示
//  transform.setRotation(q);
  //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );// 根据四元数设置旋转关系

//  ROS_INFO("Timestamp of TF_Rscamera is %f", ros::Time::now().toSec());
  //对应于回调函数中激光雷达获取的时间，而不是当前时间
  Rscamera_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                          ros::Time::now(),
                                                          lidarFrame,
                                                          cameraFrame));
}

void receiveodom(const nav_msgs::Odometry::ConstPtr& odomAfterMapped)
{
  //接收处理odom消息
  geometry_msgs::PoseWithCovarianceStamped odomPose;
  odomPose.pose.pose = odomAfterMapped->pose.pose;
  odomPose.pose.covariance = odomAfterMapped->pose.covariance;
  odomPose.header = odomAfterMapped->header;
  g_pubOdomPose.publish(odomPose);//发布odom的pose信息，数据格式为geometry_msgs::PoseWithCovarianceStamped

  //建立相机外参
//  Eigen::Matrix3d lidar2cam_rotation;
//  lidar2cam_rotation << -0.0036999,   -0.999962, 0.00784481,
//                         -0.278016, -0.00650698,  -0.960554,
//                          0.960569, -0.00573494,  -0.277981;
//  Eigen::Isometry3d e_lidar2cam(lidar2cam_rotation);//可以是旋转矩阵、四元数、旋转向量
//  lidar2cam.pretranslate(Eigen::Vector3d(0.0119567, -0.111237, -0.0959151));//添加平移向量
//  Eigen::Matrix3d cam2lidar_rotation;
//  cam2lidar_rotation << -0.0036999,   -0.278016,    0.960569,
//                         -0.999962, -0.00650698, -0.00573494,
//                        0.00784481,   -0.960554,   -0.277981;
//  Eigen::Isometry3d e_cam2lidar(cam2lidar_rotation);//可以是旋转矩阵、四元数、旋转向量
//  e_cam2lidar.pretranslate(Eigen::Vector3d(0.0612516, 0.0106824, -0.133606));//添加平移向量
  Eigen::Isometry3d e_cam2lidar = Eigen::Isometry3d::Identity();//可以是旋转矩阵、四元数、旋转向量
  tf::transformTFToEigen(tf_camera2lidar, e_cam2lidar);

  //获得odom到cameraFrame的tf关系
  nav_msgs::Odometry odomPose_Cameraframe_navmsgs(*odomAfterMapped);
  geometry_msgs::PoseWithCovarianceStamped odomPose_Cameraframe(odomPose);
  Eigen::Isometry3d e_odompose = Eigen::Isometry3d::Identity();//可以是旋转矩阵、四元数、旋转向量
  tf::poseMsgToEigen(odomPose.pose.pose, e_odompose);
  Eigen::Isometry3d e_camerapose = Eigen::Isometry3d::Identity();
  e_camerapose = e_odompose * e_cam2lidar;//velodyne转camera

  Eigen::Isometry3d e_lidar2base = Eigen::Isometry3d::Identity();//可以是旋转矩阵、四元数、旋转向量
  tf::transformTFToEigen(tf_lidar2base, e_lidar2base);
//  cout << e_lidar2base.matrix() << endl;
  e_camerapose = e_lidar2base * e_camerapose;//base转velodyne

  tf::poseEigenToMsg(e_camerapose, odomPose_Cameraframe.pose.pose);

  //  odomPose_Cameraframe_navmsgs.header = odomPose_Cameraframe.header;
  odomPose_Cameraframe_navmsgs.pose.pose = odomPose_Cameraframe.pose.pose;
  odomPose_Cameraframe_navmsgs.pose.covariance = odomPose_Cameraframe.pose.covariance;
  odomPose_Cameraframe_navmsgs.child_frame_id = cameraFrame;
  g_pubOdomPose_Cameraframe.publish(odomPose_Cameraframe_navmsgs);//发布相机坐标系下odom_pose信息，数据格式为nav_msgs::Odometry
}

void receiveRScameraPoint(const sensor_msgs::PointCloud2ConstPtr& receivePoint)
{
  sensor_msgs::PointCloud2 publishPoint;
  pcl::PointCloud<cam_PointT>::Ptr cloud (new pcl::PointCloud<cam_PointT>);
  pcl::PointCloud<cam_PointT>::Ptr cloud_filtered (new pcl::PointCloud<cam_PointT>);
  pcl::fromROSMsg(*receivePoint, *cloud);

  // 设置滤波器对象
  pcl::PassThrough<cam_PointT> pass;
  pass.setInputCloud (cloud);//设置输入点云
  pass.setFilterFieldName (FilterFieldName);//设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits (setFilterLimitsDown, setFilterLimitsUp);//设置在过滤字段的范围
  pass.filter (*cloud_filtered);//执行滤波，保存过滤结果在cloud_filtered

  pcl::toROSMsg(*cloud_filtered, publishPoint);
  publishPoint.header.frame_id = cameraFrame;
  g_pubRScameraPoint.publish(publishPoint);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "detection");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  ROS_INFO("Detection node started.");

  //创建新的转化关系
  tf::TransformBroadcaster Rscamera_broadcaster;
  tf::TransformListener tf_lidar2base_listen;

  //定义节点参数信息
  nh_param.param<string>("cameraFrame", cameraFrame, "/AreoCameraFront_depth_optical_frame");
  nh_param.param<string>("cameraPoint", camera_point_topic, "/AreoCameraFront/depth/color/points");
  nh_param.param<string>("odomFrame", odomFrame, "/odom");
  nh_param.param<string>("baseFrame", baseFrame, "/base_link");
  nh_param.param<string>("lidarFrame", lidarFrame, "/velodyne");
//  nh_param.param<double>("roll", RPY[0], 0.0);//角度制
//  nh_param.param<double>("pitch", RPY[1], 0.0);//角度制
//  nh_param.param<double>("yaw", RPY[2], 0.0);//角度制
  nh_param.param<string>("FilterFieldName", FilterFieldName, "z");//过滤字段
  nh_param.param<double>("setFilterLimitsUp", setFilterLimitsUp, 10.0);//长度范围上界
  nh_param.param<double>("setFilterLimitsDown", setFilterLimitsDown, 0.0);//长度范围下界
  nh_param.param<bool>("use_static_tf", use_static_tf, true);//过滤字段
  nh_param.param<int>("elevation_wait_time", elevation_wait_time, 10);//地图发布延时等待时间

  //定义节点订阅信息和发布信息
  g_subOdom = nh.subscribe<nav_msgs::Odometry>(odomFrame, 10, receiveodom);
  g_pubOdomPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom_pose", 10);//发布pose节点
  g_subRScameraPoint = nh.subscribe<sensor_msgs::PointCloud2>(camera_point_topic, 10, receiveRScameraPoint);
  g_pubRScameraPoint = nh.advertise<sensor_msgs::PointCloud2>("/camera_point", 10);
  g_pubOdomPose_Cameraframe = nh.advertise<nav_msgs::Odometry>("/odom_pose_cameraframe", 10);//发布pose节点
  elevation_map_hz_pub = nh.advertise<sensor_msgs::PointCloud2>("/elevation_map", 1);

  //获取tf关系
//  tf_lidar2base_listen.waitForTransform(baseFrame, lidarFrame, ros::Time(0), ros::Duration(3.0));
//  tf_lidar2base_listen.lookupTransform(baseFrame, lidarFrame, ros::Time(0), tf_lidar2base);//获取velodyne与baselink的tf转换关系

  ros::Rate rate(1000);//1000Hz，对应于1ms的延时等待查询。
  int waitforelevation = 1;//地图发布计数器
  while (ros::ok()){
    ++ waitforelevation;
    if(waitforelevation > elevation_wait_time)
    {
      sensor_msgs::PointCloud2 elevationmap;
      elevation_map_hz_pub.publish(elevationmap);
      waitforelevation = 1;
    }

    if(use_static_tf)
    {
      publichTF_Rscamera(Rscamera_broadcaster);//发布cameraFrame与velodyne的tf转换关系
    }
    try{
      tf_lidar2base_listen.lookupTransform(baseFrame, lidarFrame, ros::Time(0), tf_lidar2base);//获取velodyne与baselink的tf转换关系
      tf_lidar2base_listen.lookupTransform(lidarFrame, cameraFrame, ros::Time(0), tf_camera2lidar);//获取velodyne与baselink的tf转换关系
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
  }
  return 0;
}
