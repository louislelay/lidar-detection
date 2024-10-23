#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cluster_extraction.hpp"
#include "bounding_box.hpp"

#include "bounding_box.hpp"
#include "../pcl_cluster_extraction/cluster_extraction.hpp"
#include "../pcl_ground_removal/ground_removal.hpp"
#include "../pcl_preprocessor/filter.hpp"

using std::placeholders::_1;

class PointCloudBoundingBox : public rclcpp::Node
{
public:
    PointCloudBoundingBox() : Node("pointcloud_bounding_box")
    {
      // Create Subscriber to "input_pcl" topic
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input_pcl",
        10,
        std::bind(&PointCloudBoundingBox::pointcloud_callback, this, _1));

      // Create publisher to "output_pcl" topic
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output_pcl", 
        10);

      // Create publisher to "bounding_boxes" topic
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "bounding_boxes", 
        10);
      
      // Initialize the GroundRemoval class with a threshold for plane segmentation
      ground_removal_ = std::make_shared<GroundRemoval>(0.01);

      // Initialize the ClusterExtraction class
      cluster_extraction_ = std::make_shared<ClusterExtraction>(0.2, 5, 200);

      // Initialize the BoundingBox class
      bounding_box_ = std::make_shared<BoundingBox>();
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
    PointCloudFilter filter;                                // Filter class object
    std::shared_ptr<GroundRemoval> ground_removal_;         // Assuming GroundRemoval is defined elsewhere
    std::shared_ptr<ClusterExtraction> cluster_extraction_; // ClusterExtraction instance
    std::shared_ptr<BoundingBox> bounding_box_;             // BoundingBox instance
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudBoundingBox>());
    rclcpp::shutdown();
    return 0;
}