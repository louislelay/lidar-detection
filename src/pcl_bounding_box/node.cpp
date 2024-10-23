#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cluster_extraction.hpp"
#include "bounding_box.hpp"






// deux trois trucs manquant pr filter à ajouter et l'init des classes je me suis rendu compte je l'avais pas fait ds les autres nodes dc c'est a faire ausssi + ne pas oublier de les rendre executable ds cmakelists et package.xml





class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor()
        : Node("point_cloud_processor"),
          ground_removal_(std::make_shared<GroundRemoval>()),  // Assume GroundRemoval is defined elsewhere
          cluster_extraction_(std::make_shared<ClusterExtraction>()),
          bounding_box_(std::make_shared<BoundingBox>())
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bounding_boxes", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", 10, std::bind(&PointCloudProcessor::pointcloud_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with %d data points", msg->width * msg->height);

        try
        {
            // Preprocess and perform ground removal
            sensor_msgs::msg::PointCloud2 preprocessed_msg;
            filter.preprocessPointCloud(msg, preprocessed_msg); // Assuming you have filter defined
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(preprocessed_msg, *cloud);

            // Perform ground removal
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = ground_removal_->removeGround(cloud);

            // Perform cluster extraction
            auto clusters = cluster_extraction_->extractClusters(cloud_filtered);

            // Create a MarkerArray for bounding boxes
            visualization_msgs::msg::MarkerArray marker_array;

            // Process each cluster
            for (size_t i = 0; i < clusters.size(); ++i)
            {
                // Create a bounding box for each cluster
                auto marker = bounding_box_->createBoundingBoxMarker(clusters[i], msg->header, i);
                marker_array.markers.push_back(marker);
            }

            // Publish the bounding boxes
            marker_publisher_->publish(marker_array);
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_WARN(this->get_logger(), "Processing failed: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::shared_ptr<GroundRemoval> ground_removal_;       // Assuming GroundRemoval is defined elsewhere
    std::shared_ptr<ClusterExtraction> cluster_extraction_; // ClusterExtraction instance
    std::shared_ptr<BoundingBox> bounding_box_;           // BoundingBox instance
};