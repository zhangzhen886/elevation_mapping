#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

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

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

using namespace std;
ros::Publisher pubLaserCloudlocalMap, pub_octomap;
ros::Publisher g_pubRScamera;
ros::Subscriber g_subOdom;
ros::Subscriber subCornerPointsSharp;

float mDepthMapFactor;
double fx = 6.183770751953125e+02, fy = 6.183038330078125e+02, cx = 3.322857360839844e+02, cy = 2.4591329956054688e+02;
octomap::OcTree* tree = new octomap::OcTree(0.03);

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(const cv::Mat &dep)
{
  //根据map返回点云
    cv::Mat depth = dep;
    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || depth.type() != CV_32F)
        depth.convertTo(depth, CV_32F, mDepthMapFactor);
    PointCloud::Ptr cloud(new PointCloud());

    for (int m = 0; m < depth.rows; m ++)
    {
        for (int n = 0; n < depth.cols; n ++)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d > 10)
                continue;
            pcl::PointXYZ p;
            p.z = d;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            cloud->points.push_back(p);
        }
    }
    cloud->is_dense = false;
    cout << ", size=" << cloud->points.size() << endl;
    return cloud;
}

void GrabRGBD(const sensor_msgs::ImageConstPtr &msgD)
{
  clock_t start_time = clock();

  //根据深度信息发布点云图和octomap
    PointCloud::Ptr cloud;
    sensor_msgs::PointCloud2 cloud_ros;
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cloud = generatePointCloud(cv_ptrD->image);
    for(auto p:cloud->points)
    {
        tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }
    tree->updateInnerOccupancy();
    pcl::toROSMsg(*cloud, cloud_ros);
    cloud_ros.header.frame_id = "/camera_link";
    cloud_ros.header.stamp = ros::Time().fromSec(msgD->header.stamp.toSec());
    pubLaserCloudlocalMap.publish(cloud_ros);

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "/camera_link";
    map_msg.header.stamp = ros::Time().fromSec(msgD->header.stamp.toSec());
    if (octomap_msgs::fullMapToMsg(*tree, map_msg))
        pub_octomap.publish(map_msg);
    tree->clear();

    clock_t finish_time = clock();
    double total_time = (double)(finish_time - start_time) / CLOCKS_PER_SEC;
    std::cout << "time" << total_time * 1000 << "ms" << std::endl;
}

void publichRscameraAndTF(tf::TransformBroadcaster &Rscamera_broadcaster, string cameraFrame, double RPY[])
{
  //发布tf
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.05, 0.05, -0.1) );//设置新坐标系相对位置关系

  tf::Quaternion q;// 根据欧拉角设置旋转关系
  double pi = 3.1415926;
  q.setRPY(RPY[0] * pi / 180, RPY[1] * pi / 180, RPY[2] * pi / 180);//用弧度表示
  transform.setRotation(q);
//  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );// 根据四元数设置旋转关系

  Rscamera_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", cameraFrame));

  //发布节点消息
  std_msgs::String msg;
  std::stringstream s;
  s << "Realsense camera D435i is on!" << endl;
  msg.data = s.str();
  g_pubRScamera.publish(msg);
}

void receiveodom(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
  //接收处理odom消息
//  double roll, pitch, yaw;
//  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
//  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

//  transformAftMapped[0] = -pitch;
//  transformAftMapped[1] = -yaw;
//  transformAftMapped[2] = roll;

//  transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
//  transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
//  transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

//  transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
//  transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
//  transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

//  transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
//  transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
//  transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "detection");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
//  string depth_topic = "/AreoCameraFront/depth/image_rect_raw";

//  cout << endl << "------------------------" << endl;
//  cout << "- depth_topic: " << depth_topic << endl;

  //转换深度因子
//  mDepthMapFactor = 5000.0;
//  if (fabs(mDepthMapFactor) < 1e-5)
//      mDepthMapFactor = 1;
//  else
//      mDepthMapFactor = 1.0f / mDepthMapFactor;

  //创建新的转化关系
  tf::TransformBroadcaster Rscamera_broadcaster;

  //定义节点参数信息
  string cameraFrame;
  double RPY[3];
  nh_param.param<string>("cameraFrame", cameraFrame, "/AreoCameraFront_depth_optical_frame");
  nh_param.param<double>("roll", RPY[0], 0.0);//角度制
  nh_param.param<double>("pitch", RPY[1], 0.0);//角度制
  nh_param.param<double>("yaw", RPY[2], 0.0);//角度制

  //定义节点订阅信息和发布信息
  g_subOdom = nh.subscribe<nav_msgs::Odometry>("/odom", 10, receiveodom, ros::TransportHints().tcpNoDelay());
  g_pubRScamera = nh.advertise<std_msgs::String>("/camera_link" ,10);//发布相机节点
//  subCornerPointsSharp = nh.subscribe<sensor_msgs::Image>(depth_topic, 10, GrabRGBD, ros::TransportHints().tcpNoDelay());
//  pub_octomap = nh.advertise<octomap_msgs::Octomap>("/local_octomap", 10, true);//发布相机生成octomap
//  pubLaserCloudlocalMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 10);//发布相机生成点云

  ros::Rate rate(100);//1000Hz，对应于1ms的延时等待查询。
  while (ros::ok()){
    publichRscameraAndTF(Rscamera_broadcaster, cameraFrame, RPY);//发布camera_link与base_link的tf转换关系，以及camera_link消息
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
  }
  return 0;
}
