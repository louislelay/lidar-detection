#include "cluster_extraction.hpp"

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ClusterExtraction::extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // Create the segmentation object for clustering
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); // Set the distance threshold
    ec.setMinClusterSize(min_cluster_size_);    // Set the minimum cluster size
    ec.setMaxClusterSize(max_cluster_size_);    // Set the maximum cluster size

    // Create a KdTree for efficient searching
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    // Vector to hold cluster indices
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    // Create separate point clouds for each cluster
    for (const auto &indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &index : indices.indices)
        {
            cluster->points.push_back(cloud->points[index]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1; // Unorganized point cloud
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    return clusters;
}