#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "cluster_extraction.hpp"
#include "../pcl_ground_removal/ground_removal.hpp"
#include "../pcl_preprocessor/filter.hpp"

using std::placeholders::_1;

class PointCloudSegmentation : public rclcpp::Node
{
  public:
    PointCloudSegmentation() : Node("pointcloud_segmentation")
    {
      // Create Subscriber to "input_pcl" topic
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input_pcl",
        10,
        std::bind(&PointCloudSegmentation::pointcloud_callback, this, _1));

      // Create publisher to "output_pcl" topic
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output_pcl", 
        10);
      
      // Initialize the GroundRemoval class with a threshold for plane segmentation
      ground_removal_ = std::make_shared<GroundRemoval>(0.01);

      // Initialize the ClusterExtraction class
      cluster_extraction_ = std::make_shared<ClusterExtraction>(0.2, 5, 200);
    }

  private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with %d data points", msg->width * msg->height);

      try
      {
        // Create output message
        sensor_msgs::msg::PointCloud2 preprocessed_msg;

        filter.preprocessPointCloud(msg, preprocessed_msg);

        // Convert ROS2 PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(preprocessed_msg, *cloud);

        // Perform ground removal in one line using the GroundRemoval class
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = ground_removal_->removeGround(cloud);

        // Perform cluster extraction
        auto clusters = cluster_extraction_->extractClusters(cloud_filtered);

        RCLCPP_INFO(this->get_logger(), "Extracted %lu clusters", clusters.size());

        // Process and publish each cluster (optional)
        for (const auto &cluster : clusters)
        {
          sensor_msgs::msg::PointCloud2 cluster_msg;
          pcl::toROSMsg(*cluster, cluster_msg);
          cluster_msg.header = msg->header; // Set the same header as input

          // Log the cluster information
          RCLCPP_INFO(this->get_logger(), "Sent PointCloud2 with %d data points", cluster_msg.width * cluster_msg.height);

          // Publish each cluster (you might want to create separate topics for clusters)
          publisher_->publish(cluster_msg);
          
        }
      }
      catch (const std::runtime_error &e)
      {
        RCLCPP_WARN(this->get_logger(), "%s", e.what());
      }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    PointCloudFilter filter;  // Filter class object
    std::shared_ptr<GroundRemoval> ground_removal_; // Ground removal class object
    std::shared_ptr<ClusterExtraction> cluster_extraction_; // ClusterExtraction instance
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSegmentation>());
    rclcpp::shutdown();
    return 0;
}