#include "ground_removal.hpp"

GroundRemoval::GroundRemoval(double distance_threshold = 0.01)
{
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setDistanceThreshold(distance_threshold);
}


// Function to remove ground
pcl::PointCloud<pcl::PointXYZ>::Ptr GroundRemoval::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Perform RANSAC-based plane segmentation
    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coefficients);

    // Check if a plane was detected
    if (inliers->indices.empty())
    {
        throw std::runtime_error("No ground plane detected.");
    }

    // Extract points that are not part of the ground plane
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    return cloud_filtered;
}


