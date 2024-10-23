#ifndef FILTER_HPP
#define FILTER_HPP

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

class PointCloudFilter {
public:
    // Constructor
    PointCloudFilter();

    // Preprocessing function
    void preprocessPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, sensor_msgs::PointCloud2& output_cloud);
};

#endif // FILTER_HPP