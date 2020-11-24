#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Subscriber grid_sub;
ros::Publisher occupancy_grid_pub;
float data_min;
float data_max;
std::string elevation_layer;
std::string traversability_layer;
std::string grid_sub_topic;
std::string occupancy_grid_pub_topic;

void Grid2occupancy(const grid_map_msgs::GridMap& grid_map_message)
{
    ROS_INFO_THROTTLE(10.0, "Grid map visualization received a map (timestamp %f) for visualization.",
              grid_map_message.info.header.stamp.toSec());
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(grid_map_message, gridMap);
    nav_msgs::OccupancyGrid occupancyGrid;

    occupancyGrid.header.frame_id = gridMap.getFrameId();
    occupancyGrid.header.stamp.fromNSec(gridMap.getTimestamp());
    occupancyGrid.info.map_load_time = occupancyGrid.header.stamp;  // Same as header stamp as we do not load the map.
    occupancyGrid.info.resolution = gridMap.getResolution();
    occupancyGrid.info.width = gridMap.getSize()(0);
    occupancyGrid.info.height = gridMap.getSize()(1);
    grid_map::Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
    occupancyGrid.info.origin.position.x = position.x();
    occupancyGrid.info.origin.position.y = position.y();
    occupancyGrid.info.origin.position.z = 0.0;
    occupancyGrid.info.origin.orientation.x = 0.0;
    occupancyGrid.info.origin.orientation.y = 0.0;
    occupancyGrid.info.origin.orientation.z = 0.0;
    occupancyGrid.info.origin.orientation.w = 1.0;
    size_t nCells = gridMap.getSize().prod();
    occupancyGrid.data.resize(nCells);

    // Occupancy probabilities are in the range [0,127]. Unknown is -1.
    const float cellMin = 0;
    const float cellMax = 100;
    const float cellRange = cellMax - cellMin;

    for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
        float value = (gridMap.at(traversability_layer, *iterator) - data_min) / (data_max - data_min);
        if (std::isnan(value) || (!gridMap.isValid(*iterator, elevation_layer)))
            value = -1;
        else
            value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
        size_t index = grid_map::getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
        // Reverse cell order because of different conventions between occupancy grid and grid map.
        occupancyGrid.data[nCells - index - 1] = value;
    }

    occupancy_grid_pub.publish(occupancyGrid);
}

void ReadParameter(ros::NodeHandle& nh_param)
{
    nh_param.param<std::string>("grid_sub_topic", grid_sub_topic, "/grid_map_filtered");
    nh_param.param<std::string>("occupancy_grid_pub_topic", occupancy_grid_pub_topic, "/occupancy_grid_traversability");
    nh_param.param<std::string>("elevation_layer", elevation_layer, "elevation");
    nh_param.param<std::string>("traversability_layer", traversability_layer, "traversability");
    nh_param.param<float>("data_min", data_min, 0.0);
    nh_param.param<float>("data_max", data_max, 1.0);
}

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_to_occupancy");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_param("~");
    ReadParameter(nh_param);

    grid_sub = nh.subscribe(grid_sub_topic, 10, Grid2occupancy);
    occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(occupancy_grid_pub_topic, 10, true);

    // Work with grid map in a loop.
    ros::Rate rate(1000.0);

    while (nh.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}