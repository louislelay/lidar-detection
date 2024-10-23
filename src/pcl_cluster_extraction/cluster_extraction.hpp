#ifndef CLUSTER_EXTRACTION_HPP
#define CLUSTER_EXTRACTION_HPP

#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

class ClusterExtraction
{
    public:
        // Constructor
        ClusterExtraction(float cluster_tolerance = 0.1, int min_cluster_size = 5, int max_cluster_size = 3000) : 
            cluster_tolerance_(cluster_tolerance),
            min_cluster_size_(min_cluster_size),
            max_cluster_size_(max_cluster_size) {}

        // Function to extract cluster
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    private:
        float cluster_tolerance_;
        int min_cluster_size_;
        int max_cluster_size_;
};

#endif // CLUSTER_EXTRACTION_HPP