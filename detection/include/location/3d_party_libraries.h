#ifndef _3D_PARTY_LIBRARIES_H_
#define _3D_PARTY_LIBRARIES_H_

#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <boost/thread/thread.hpp>
#include <eigen3/Eigen/Core>

#include <tf/tf.h>
using namespace std;
//数据类型不影响ndt
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

void downsampleCloud(const VPointCloud::Ptr in_cloud_ptr, \
                     VPointCloud::Ptr out_cloud_ptr, \
                     float in_leaf_size=0.5)
{
    pcl::VoxelGrid<VPoint> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
}

void filterGroundPlane(const VPointCloud& pc, \
                       VPointCloud& out_cloud_withoutGround_ptr, \
                       VPointCloud& out_cloud_Ground_ptr)
{
    if (pc.size() < 50){
      printf("Pointcloud in OctomapServer too small, skipping ground plane extraction\r\n");
      out_cloud_withoutGround_ptr = pc;
    } else {
      // plane detection for ground plane removal:
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      // Create the segmentation object and set up:
      pcl::SACSegmentation<VPoint> seg;//创建分割对象
      seg.setOptimizeCoefficients (true);//设置对估计的模型参数进行优化处理
      // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(5000); //设置最大迭代次数
      seg.setDistanceThreshold (0.1);//设置判断是否为模型内点的距离阈值
      seg.setAxis(Eigen::Vector3f(0,0,1));
      seg.setEpsAngle(15/180*3.14);


      VPointCloud cloud_filtered(pc);
      // Create the filtering object
      pcl::ExtractIndices<VPoint> extract; //创建点云提取对象
      bool groundPlaneFound = false;

      while(cloud_filtered.size() > 10 && !groundPlaneFound){
        seg.setInputCloud(cloud_filtered.makeShared());
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0){
          cout << ("PCL segmentation did not find any plane.\r\n");
          break;
        }
//为了处理点云中包含多个模型，我们在一个循环中执行该过程，并在每次模型被提取后，我们保存剩余的点，进行迭代。模型内点通过分割过程获取，如下
        extract.setInputCloud(cloud_filtered.makeShared());
        extract.setIndices(inliers);//设置分割后的内点为需要提取的点集

        if (std::abs(coefficients->values.at(3)) < 0.2){
          printf("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f\r\n", inliers->indices.size(), cloud_filtered.size(),
                    coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
          extract.setNegative (false);//设置提取内点而非外点
          extract.filter (out_cloud_Ground_ptr);//提取输出存储到cloud_p

          // remove ground points from full pointcloud:
          // workaround for PCL bug:
          if(inliers->indices.size() != cloud_filtered.size()){
            extract.setNegative(true);//设置提取外点
            VPointCloud cloud_out;
            extract.filter(cloud_out);//提取外点
            out_cloud_withoutGround_ptr += cloud_out;
            cloud_filtered = cloud_out;
          }

          groundPlaneFound = true;
        } else{
          printf("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f\r\n", inliers->indices.size(), cloud_filtered.size(),
                    coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
          pcl::PointCloud<VPoint> cloud_out;
          extract.setNegative (false);
          extract.filter(cloud_out);
          out_cloud_withoutGround_ptr +=cloud_out;
          if(inliers->indices.size() != cloud_filtered.size()){
            extract.setNegative(true);
            cloud_out.points.clear();
            extract.filter(cloud_out);
            cloud_filtered = cloud_out;
          } else{
            cloud_filtered.points.clear();
          }
        }

      }
      // TODO: also do this if overall starting pointcloud too small?
      if (!groundPlaneFound){ // no plane found or remaining points too small
        printf("No ground plane found in scan\r\n");

        // do a rough fitlering on height to prevent spurious obstacles
        pcl::PassThrough<VPoint> second_pass;
        second_pass.setFilterFieldName("z");
        second_pass.setFilterLimits(-0.5, 0.5);
        second_pass.setInputCloud(pc.makeShared());
        second_pass.filter(out_cloud_Ground_ptr);

        second_pass.setFilterLimitsNegative (true);
        second_pass.filter(out_cloud_withoutGround_ptr);
      }

    }

}

//会破坏结构化数据，注意
void removePointsUpTo(const VPointCloud::Ptr in_cloud_ptr, VPointCloud::Ptr out_cloud_ptr, float min_distance=1.0, float max_distance=100.0)
{
  out_cloud_ptr->header = in_cloud_ptr->header;
  out_cloud_ptr->clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); ++i)
  {
    //使用计算量更少的方法
    if(!isnan(in_cloud_ptr->points[i].x));
    float fabsX = fabs(in_cloud_ptr->points[i].x);
    float fabsY = fabs(in_cloud_ptr->points[i].y);
    if(fabsX>min_distance && fabsY>min_distance && fabsX<max_distance && fabsY<max_distance)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }


//    double len2 = in_cloud_ptr->points[i].x * in_cloud_ptr->points[i].x +
//        in_cloud_ptr->points[i].y * in_cloud_ptr->points[i].y +
//        in_cloud_ptr->points[i].z * in_cloud_ptr->points[i].z;

//    if (min_distance * min_distance < len2 && max_distance * max_distance > len2)
//        //&& in_cloud_ptr->points[i].z < 0)
//    {
//      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
//    }
  }
}
#endif
