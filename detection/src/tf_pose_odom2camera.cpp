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
#include <message_filters/cache.h>
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

string cameraFrame;
ros::Subscriber g_subOdom;
ros::Publisher g_pubOdomPose_Cameraframe;
tf::StampedTransform g_tf_camera2velodyne;

void receiveodom(const nav_msgs::Odometry::ConstPtr& odomAfterMapped)
{
  //接收处理odom消息
  geometry_msgs::PoseWithCovarianceStamped odomPose;
  odomPose.pose.pose = odomAfterMapped->pose.pose;
  odomPose.pose.covariance = odomAfterMapped->pose.covariance;
  odomPose.header = odomAfterMapped->header;


  //获得odom到cameraFrame的tf关系
  geometry_msgs::PoseWithCovarianceStamped odomPose_Cameraframe(odomPose);
  geometry_msgs::TransformStamped tf_msgs;
  tf::transformStampedTFToMsg(g_tf_camera2velodyne, tf_msgs);
  odomPose_Cameraframe.header.frame_id = cameraFrame;
  odomPose_Cameraframe.pose.pose.position.x += tf_msgs.transform.translation.x;
  odomPose_Cameraframe.pose.pose.position.y += tf_msgs.transform.translation.y;
  odomPose_Cameraframe.pose.pose.position.z += tf_msgs.transform.translation.z;
  odomPose_Cameraframe.pose.pose.orientation.x *= tf_msgs.transform.rotation.x;
  odomPose_Cameraframe.pose.pose.orientation.y *= tf_msgs.transform.rotation.y;
  odomPose_Cameraframe.pose.pose.orientation.z *= tf_msgs.transform.rotation.z;
  odomPose_Cameraframe.pose.pose.orientation.w *= tf_msgs.transform.rotation.w;
//  tf::TransformListener tf_listener;
//  geometry_msgs::PoseStamped p_odomPose;
//  p_odomPose.header = odomPose.header;
//  p_odomPose.header.frame_id = "velodyne";
//  p_odomPose.pose = odomPose.pose.pose;
//  geometry_msgs::PoseStamped p_odomPose_Cameraframe(p_odomPose);
//  try
//  {
//    tf_listener.transformPose(cameraFrame, p_odomPose, p_odomPose_Cameraframe);
//  }
//  catch( tf::TransformException ex)
//  {
//    ROS_WARN("transfrom exception : %s",ex.what());
//    return;
//  }
//  odomPose_Cameraframe.header = p_odomPose_Cameraframe.header;
//  odomPose_Cameraframe.pose.pose = p_odomPose_Cameraframe.pose;
//  odomPose_Cameraframe.pose.covariance = odomPose.pose.covariance;
//  ROS_INFO_STREAM(odomPose_Cameraframe);
  g_pubOdomPose_Cameraframe.publish(odomPose_Cameraframe);//发布相机坐标系下odom_pose信息，数据格式为geometry_msgs::PoseWithCovarianceStamped
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_pose_odom2camera");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  ROS_INFO("tf_pose_odom2camera node started.");

  //定义节点参数信息
  nh_param.param<string>("cameraFrame", cameraFrame, "/AreoCameraFront_depth_optical_frame");

  //调试
  g_subOdom = nh.subscribe<nav_msgs::Odometry>("/odom", 10, receiveodom);
  g_pubOdomPose_Cameraframe = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom_pose_cameraframe", 10);//发布pose节点

  tf::TransformListener tf_listener;
  //先获得cameraFrame到velodyne的tf转换关系
  tf_listener.waitForTransform(cameraFrame, "/velodyne", ros::Time(0), ros::Duration(1.0));
  tf_listener.lookupTransform(cameraFrame, "/velodyne", ros::Time(0), g_tf_camera2velodyne);

  ros::Rate rate(10);//1000Hz，对应于1ms的延时等待查询。
  while (ros::ok()){
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
  }
  return 0;
}
