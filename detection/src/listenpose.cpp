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

ros::Time lastPointCloudUpdateTime_;
message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;//调试

void pointCloudCallback(const sensor_msgs::PointCloud2& rawPointCloud)
{
//  lastPointCloudUpdateTime_.fromNSec();
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(robotPoseCache_.getLatestTime());
  if(poseMessage){
    ROS_INFO("get pose time: %f", poseMessage->header.stamp.toSec());
  }
  ROS_INFO("get time: %f", robotPoseCache_.getLatestTime().toSec());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "listenpose");
  ros::NodeHandle nh;
  ROS_INFO("Listenpose node started.");
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;
  ros::Subscriber pointCloudSubscriber_;

  //调试
  robotPoseSubscriber_.subscribe(nh, "/odom_pose", 1);
  robotPoseCache_.connectInput(robotPoseSubscriber_);
  robotPoseCache_.setCacheSize(200);
  pointCloudSubscriber_ = nh.subscribe("/AreoCameraFront/depth/color/points", 1, pointCloudCallback);

  ros::Rate rate(10);//1000Hz，对应于1ms的延时等待查询。
  while (ros::ok()){
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
  }
  return 0;
}
