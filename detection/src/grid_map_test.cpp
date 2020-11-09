#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//using namespace grid_map;
// Create grid map.
grid_map::GridMap map({"elevation"});
ros::Publisher grid_pub;
ros::Subscriber point_sub;

std::string point_sub_topic;
std::string grid_pub_topic;
std::string grid_frame_id;
float grid_map_size_x;
float grid_map_size_y;
float grid_map_resolution;

//void Point2grid_with_elevation(const sensor_msgs::PointCloud2ConstPtr& receivePoint)
//{
//    ros::Time timestamp = ros::Time::now();
//
//    sensor_msgs::PointCloud2 publishPoint;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromROSMsg(*receivePoint, *pointCloud);
//
//    Eigen::VectorXf pointCloudVariances;
//    if (pointCloud->size() != pointCloudVariances.size()) {
//        ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.",
//                  (int) pointCloud->size(), (int) pointCloudVariances.size());
//    }
//
////    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
//    for (unsigned int i = 0; i < pointCloud->size(); ++i){
//        auto& point = pointCloud->points[i];
//        grid_map::Index index;
//        grid_map::Position position(point.x, point.y);
//
//        if (!map.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.
//
//        auto& elevation = map.at("elevation", index);
//        auto& variance = map.at("variance", index);
//        auto& horizontalVarianceX = map.at("horizontal_variance_x", index);
//        auto& horizontalVarianceY = map.at("horizontal_variance_y", index);
//        auto& horizontalVarianceXY = map.at("horizontal_variance_xy", index);
//        auto& color = map.at("color", index);
//        auto& time = map.at("time", index);
//        auto& lowestScanPoint = map.at("lowest_scan_point", index);
//        auto& sensorXatLowestScan = map.at("sensor_x_at_lowest_scan", index);
//        auto& sensorYatLowestScan = map.at("sensor_y_at_lowest_scan", index);
//        auto& sensorZatLowestScan = map.at("sensor_z_at_lowest_scan", index);
//
//        const float& pointVariance = pointCloudVariances(i);
//        const float scanTimeSinceInitialization = (timestamp - initialTime_).toSec();
//
//        if (!map.isValid(index)) {
//            // No prior information in elevation map, use measurement.
//            elevation = point.z;variance = pointVariance;
//            horizontalVarianceX = minHorizontalVariance_;
//            horizontalVarianceY = minHorizontalVariance_;
//            horizontalVarianceXY = 0.0;
//            colorVectorToValue(point.getRGBVector3i(), color);
//            continue;
//        }
//
//        // Deal with multiple heights in one cell.
//        const double mahalanobisDistance = fabs(point.z - elevation) / sqrt(variance);
//        if (mahalanobisDistance > mahalanobisDistanceThreshold_) {
//            if (scanTimeSinceInitialization - time <= scanningDuration_ && elevation > point.z) {
//                // Ignore point if measurement is from the same point cloud (time comparison) and
//                // if measurement is lower then the elevation in the map.
//            } else if (scanTimeSinceInitialization - time <= scanningDuration_) {
//                // If point is higher.
//                elevation = point.z;
//                variance = pointVariance;
//            } else {
//                variance += multiHeightNoise_;
//            }
//            continue;
//        }
//
//        // Store lowest points from scan for visibility checking.
//        const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance); // 3 sigma.
//        if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint){
//            lowestScanPoint = pointHeightPlusUncertainty;
//            const Position3 sensorTranslation(transformationSensorToMap.translation());
//            sensorXatLowestScan = sensorTranslation.x();
//            sensorYatLowestScan = sensorTranslation.y();
//            sensorZatLowestScan = sensorTranslation.z();
//        }
//
//        // Fuse measurement with elevation map data.
//        elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
//        variance = (pointVariance * variance) / (pointVariance + variance);
//        // TODO Add color fusion.
//        colorVectorToValue(point.getRGBVector3i(), color);
//        timestamp = scanTimeSinceInitialization;
//
//        // Horizontal variances are reset.
//        horizontalVarianceX = minHorizontalVariance_;
//        horizontalVarianceY = minHorizontalVariance_;
//        horizontalVarianceXY = 0.0;
//    }
//
//    // Publish grid map.
//    map.setTimestamp(timestamp.toNSec());
//    grid_map_msgs::GridMap message;
//    grid_map::GridMapRosConverter::toMessage(map, message);
//    grid_pub.publish(message);
//    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
//
//}

void Point2grid(const sensor_msgs::PointCloud2ConstPtr& receivePoint)
{
    ros::Time timestamp = ros::Time::now();

    sensor_msgs::PointCloud2 publishPoint;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*receivePoint, *pointCloud);


//    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    for (unsigned int i = 0; i < pointCloud->size(); ++i){
        auto& point = pointCloud->points[i];
        grid_map::Index index;
        grid_map::Position position(point.x, point.y);

        if (!map.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.

        auto& elevation = map.at("elevation", index);

//        if (!map.isValid(index)) {
//            // No prior information in elevation map, use measurement.
//            elevation = point.z;
//            continue;
//        }
        elevation = point.z;
    }

    // Publish grid map.
    map.setTimestamp(timestamp.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    grid_pub.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

}

void ReadParameter(ros::NodeHandle& nh_param)
{
    nh_param.param<std::string>("point_sub_topic", point_sub_topic, "/map_points");
    nh_param.param<std::string>("grid_pub_topic", grid_pub_topic, "/grid_map");
    nh_param.param<std::string>("grid_frame_id", grid_frame_id, "/lidar_odom");
    nh_param.param<float>("grid_map_size_x", grid_map_size_x, 20.0);
    nh_param.param<float>("grid_map_size_y", grid_map_size_y, 20.0);
    nh_param.param<float>("grid_map_resolution", grid_map_resolution, 0.2);
}

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_test");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_param("~");
    ReadParameter(nh_param);

    point_sub = nh.subscribe<sensor_msgs::PointCloud2>(point_sub_topic, 10, Point2grid);
    grid_pub = nh.advertise<grid_map_msgs::GridMap>(grid_pub_topic, 1, true);


    // Initial grid map.
    map.setFrameId(grid_frame_id);
    map.setGeometry(grid_map::Length(grid_map_size_x, grid_map_size_y), grid_map_resolution);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(),
             map.getSize()(0), map.getSize()(1));

    // Work with grid map in a loop.
    ros::Rate rate(1000.0);

    while (nh.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
