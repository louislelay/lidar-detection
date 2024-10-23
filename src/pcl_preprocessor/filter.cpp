#include "filter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// Constructor
PointCloudFilter::PointCloudFilter() {}

// Preprocess point cloud (e.g., using a VoxelGrid filter for downsampling)
void PointCloudFilter::preprocessPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, sensor_msgs::PointCloud2& output_cloud) {
    // Convert ROS PointCloud2 message to PCL data type
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*input_cloud, *pcl_cloud);

    // Apply VoxelGrid filter for downsampling
    pcl::PCLPointCloud2::Ptr pcl_filtered_cloud(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Adjust voxel size for downsampling
    voxel_filter.filter(*pcl_filtered_cloud);

    // Convert filtered PCL data back to ROS PointCloud2 message
    pcl_conversions::fromPCL(*pcl_filtered_cloud, output_cloud);
}
