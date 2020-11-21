#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

//重建相关
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
ros::Publisher rawmap_pub;
ros::Publisher fusemap_pub;
ros::Subscriber rawmap_sub;
typedef pcl::PointXYZ cam_PointT;
typedef pcl::PointXYZRGB cam_fuse_PointT;
typedef pcl::PointXYZRGBNormal cam_SurfelT;
pcl::PointCloud<cam_PointT>::Ptr rawmap_ptr (new pcl::PointCloud<cam_PointT>);
//pcl::PointCloud<pcl::PointXYZI>::Ptr rawmap_ptr_time (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<cam_PointT>::Ptr rawmap_ptr_filtered (new pcl::PointCloud<cam_PointT>);
pcl::PointCloud<cam_fuse_PointT>::Ptr fusemap_ptr_filtered (new pcl::PointCloud<cam_fuse_PointT>);

// 节点参数
string cameraFrame;
string mapFrame;
string FilterFieldName;
string camera_point_topic;
double setFilterLimitsUp;
double setFilterLimitsDown;
double setFilterheight;
bool setOutlierFilter;
int setOutlierFilterMeanK;
double setOutlierFilterThresh;
float voxelsize;
float roshz;
int wait_surfel_finished;
int setPointNumLimit;

void fuze_Depth_Pose(const sensor_msgs::PointCloud2ConstPtr& depth, const nav_msgs::OdometryConstPtr& pose){
  sensor_msgs::PointCloud2 syn_depth = *depth;
  nav_msgs::Odometry syn_camerapose = *pose;
  ros::Time fuse_start_time = ros::Time::now();
  ros::Time point_sync = syn_depth.header.stamp;
//  cout << "point time now is : " << point_sync.toSec() << endl;

  //转换数据格式为pcl
  pcl::PointCloud<cam_PointT>::Ptr cur_cloud (new pcl::PointCloud<cam_PointT>);
  pcl::PointCloud<cam_PointT>::Ptr cur_cloud_filtered (new pcl::PointCloud<cam_PointT>);

  pcl::fromROSMsg(syn_depth, *cur_cloud);

//  //时间赋值
//  cur_cloud->clear();
//  for(int i = 0; i < syn_depth.data.size(); ++i)
//  {
//    cam_PointT current_point;
//    current_point.x = syn_depth.data[i].x;
//    current_point.y = syn_depth.data[i].y;
//    current_point.z = syn_depth.data[i].z;
//    current_point.intensity = syn_depth.header.stamp.toSec();
//    cur_cloud->push_back(current_point);
//  }

  // 直通滤波
  pcl::PassThrough<cam_PointT> pass;
  pass.setInputCloud (cur_cloud);//设置输入点云
  pass.setFilterFieldName (FilterFieldName);//设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits (setFilterLimitsDown, setFilterLimitsUp);//设置在过滤字段的范围
  pass.filter (*cur_cloud_filtered);//执行滤波，保存过滤结果在cloud_filtered

  //坐标变换
  pcl::PointCloud<cam_PointT>::Ptr tran_cloud (new pcl::PointCloud<cam_PointT>);
  Eigen::Affine3d e_campose = Eigen::Affine3d::Identity();//仿射变换矩阵初始化
  tf::poseMsgToEigen(syn_camerapose.pose.pose, e_campose);
//  e_campose.makeAffine();
  pcl::transformPointCloud(*cur_cloud_filtered, *tran_cloud, e_campose);

  *rawmap_ptr += *tran_cloud;//直接累加(关键步骤！！！)
//  *rawmap_ptr = *tran_cloud;//不累加，显示每帧数据

  //去除离群点
  pcl::PointCloud<cam_PointT>::Ptr rawmap_ptr_removed (new pcl::PointCloud<cam_PointT>);
  if(setOutlierFilter){
    pcl::StatisticalOutlierRemoval<cam_PointT> outlier;
    outlier.setInputCloud(rawmap_ptr);
    outlier.setMeanK(setOutlierFilterMeanK);
    outlier.setStddevMulThresh(setOutlierFilterThresh);
    outlier.filter(*rawmap_ptr_removed);
  }
  else{
    *rawmap_ptr_removed = *rawmap_ptr;
  }

  //体素滤波
  pcl::VoxelGrid<cam_PointT> voxel;
  voxel.setInputCloud(rawmap_ptr_removed);//设置输入点云
  voxel.setLeafSize(voxelsize, voxelsize, voxelsize);
  voxel.filter(*rawmap_ptr_filtered);//执行滤波

  //显示点云数据量大小
//  cout << "cur_cloud_filtered_size: " << tran_cloud->size() << endl;
//  cout << "cur_cloud_filtered_totalsize: " << rawmap_ptr_filtered->size() << endl;

  //转换数据格式为ROS
  sensor_msgs::PointCloud2 rawmap_sensor;
  pcl::toROSMsg(*rawmap_ptr_filtered, rawmap_sensor);
  rawmap_sensor.header.frame_id = mapFrame;
  rawmap_sensor.header.stamp = syn_depth.header.stamp;

  rawmap_pub.publish(rawmap_sensor);//发布
//  cout << "raw map published" << endl;

  //颜色赋值
  fusemap_ptr_filtered->clear();
  for(int i = rawmap_ptr_filtered->size(); i >= 0; --i)
  {
    cam_fuse_PointT current_point;
    current_point.x = rawmap_ptr_filtered->points[i].x;
    current_point.y = rawmap_ptr_filtered->points[i].y;
    current_point.z = rawmap_ptr_filtered->points[i].z;
    if(current_point.z > setFilterheight)
    {
      current_point.r = 255;
      current_point.g = 0;
      current_point.b = 0;
    }
    else
    {
      current_point.r = 0;
      current_point.g = 255;
      current_point.b = 0;
    }
    fusemap_ptr_filtered->push_back(current_point);
    if(fusemap_ptr_filtered->size() > setPointNumLimit)
    {
      break;
    }
  }
//  pcl::copyPointCloud(*rawmap_ptr, *input_cloudmap);//转换点云格式
  sensor_msgs::PointCloud2 fusemap_sensor;
  pcl::toROSMsg(*fusemap_ptr_filtered, fusemap_sensor);
  fusemap_sensor.header.frame_id = mapFrame;
  fusemap_sensor.header.stamp = ros::Time(0);
  fusemap_pub.publish(fusemap_sensor);//发布融合地图

  //显示大小和时间
  ROS_INFO_THROTTLE(10.0, "fused cloudmap size: %lu, fuse map cost time: %f s", fusemap_ptr_filtered->size(), (ros::Time::now() - fuse_start_time).toSec());
  }

void surfel_mapping()
{
  //计算法向量
  double mls_radius = 0.05;
  double polynomial_order = 2;
  pcl::PointCloud<cam_SurfelT>::Ptr surfels (new pcl::PointCloud<cam_SurfelT>);
  pcl::MovingLeastSquares<cam_fuse_PointT, cam_SurfelT> mls;
  pcl::search::KdTree<cam_fuse_PointT>::Ptr tree (new pcl::search::KdTree<cam_fuse_PointT>);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(mls_radius);
  mls.setComputeNormals(true);
  mls.setSqrGaussParam(mls_radius * mls_radius);
  mls.setPolynomialFit(polynomial_order > 1);
  mls.setPolynomialOrder(polynomial_order);
  mls.setInputCloud(fusemap_ptr_filtered);
  mls.process(*surfels);

  cout << "normal finished!" << endl;

  //三角化
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh);
  pcl::search::KdTree<cam_SurfelT>::Ptr s_tree (new pcl::search::KdTree<cam_SurfelT>);
  s_tree->setInputCloud(surfels);
  pcl::GreedyProjectionTriangulation<cam_SurfelT> gp3;
  gp3.setSearchRadius(1.0);//最大边长度
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(2000);
  gp3.setMaximumSurfaceAngle(M_PI / 4);
  gp3.setMaximumAngle(2 * M_PI / 3);
  gp3.setMinimumAngle(M_PI / 18);
  gp3.setNormalConsistency(true);
  gp3.setInputCloud(surfels);
  gp3.setSearchMethod(s_tree);
  gp3.reconstruct(*mesh);

  //显示
  cout << "mesh finished!" << endl;
  pcl::visualization::PCLVisualizer vis;
  vis.addPolylineFromPolygonMesh(*mesh, "mesh frame");
  vis.addPolygonMesh(*mesh, "mesh");
  vis.resetCamera();
//  vis.close();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "rawmapping");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  ROS_INFO("RawMapping node started.");

  //定义节点参数信息
  nh_param.param<string>("cameraFrame", cameraFrame, "/AreoCameraFront_depth_optical_frame");
  nh_param.param<string>("cameraPoint", camera_point_topic, "/AreoCameraFront/depth/color/points");
  nh_param.param<string>("mapFrame", mapFrame, "/odom");//地图发布时的frame
  nh_param.param<string>("FilterFieldName", FilterFieldName, "z");//过滤字段
  nh_param.param<double>("setFilterLimitsUp", setFilterLimitsUp, 10.0);//长度范围上界
  nh_param.param<double>("setFilterLimitsDown", setFilterLimitsDown, 0.0);//长度范围下界
  nh_param.param<double>("setFilterheight", setFilterheight, 2.0);//高度阈值
  nh_param.param<bool>("setOutlierFilter", setOutlierFilter, false);//进行离群点去除
  nh_param.param<int>("setOutlierFilterMeanK", setOutlierFilterMeanK, 100);//k近邻点数
  nh_param.param<double>("setOutlierFilterThresh", setOutlierFilterThresh, 1.0);//离群点阈值
  nh_param.param<float>("voxelsize", voxelsize, 0.1);//体素大小
  nh_param.param<float>("roshz", roshz, 10);//延时等待查询频率
  nh_param.param<int>("wait_surfel_finished", wait_surfel_finished, 100);//延时等待查询重建图
  nh_param.param<int>("setPointNumLimit", setPointNumLimit, 1000000);//限制点的数量

  //定义节点订阅信息和发布信息
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub(nh, "/odom_pose_cameraframe", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(nh, camera_point_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> pose_depth_policy;//时间戳对齐
  message_filters::Synchronizer<pose_depth_policy> sync(pose_depth_policy(10), depth_sub, pose_sub);//消息队列大小为10
  sync.registerCallback(boost::bind(&fuze_Depth_Pose, _1, _2));

  rawmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/rawmap", 1);//发布原始地图
  fusemap_pub = nh.advertise<sensor_msgs::PointCloud2>("/fusemap", 1);//发布融合地图

//  surfel_pub = nh.subscribe<sensor_msgs::>

  int waitforsurfel = 0;
  ros::Rate rate(roshz);//1000Hz对应于1ms的延时等待查询。
  while (ros::ok()){
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数
    if(waitforsurfel >= wait_surfel_finished)//延时等待三角化重建
    {
//      surfel_mapping();
      waitforsurfel = 0;
    }
    ++waitforsurfel;

    rate.sleep();//以ros::Rate来延时
  }
  return 0;
}
