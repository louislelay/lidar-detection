#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ground_removal.hpp"
#include "../pcl_preprocessor/filter.hpp"

using std::placeholders::_1;

class PointCloudGroundRemoval : public rclcpp::Node
{
  public:
    PointCloudGroundRemoval() : Node("pointcloud_ground_removal")
    {
      // Create Subscriber to "input_pcl" topic
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input_pcl",
        10,
        std::bind(&PointCloudGroundRemoval::pointcloud_callback, this, _1));

      // Create publisher to "output_pcl" topic
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output_pcl", 
        10);

      // Initialize the GroundRemoval class with a threshold for plane segmentation
      ground_removal_ = std::make_shared<GroundRemoval>(0.01);
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

        // Convert the filtered PCL point cloud back to ROS2 PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;

        // Publish the filtered point cloud
        publisher_->publish(output_msg);
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudGroundRemoval>());
    rclcpp::shutdown();
    return 0;
}