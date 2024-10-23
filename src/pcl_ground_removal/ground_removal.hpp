#ifndef GROUND_REMOVAL_HPP
#define GROUND_REMOVAL_HPP

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class GroundRemoval
{
    public:
        // Constructor
        GroundRemoval(double distance_threshold = 0.01);

        // Function to remove ground
        pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    
    private:
        pcl::SACSegmentation<pcl::PointXYZ> seg_;
};

#endif // GROUND_REMOVAL_HPP