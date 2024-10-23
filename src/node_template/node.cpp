#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;


class PointCloudProcessor : public rclpp::Node
{
  public:
    PointCloudProcessor() : Node("pointcloud_processor")
    {
      // Create Subscriber to "input_pcl" topic
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input_pcl",
        10,
        std::bind(&PointCloudProcessor::pointcloud_callback, this, _1));

      // Create publisher to "output_pcl" topic
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output_pcl", 
        10);
    }

  private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with %d data points", msg->width * msg->height);

      // Processing the point cloud data
      auto output_msg = *msg;

      /* HERE TO MODIFY THE PointCloud2 DATA AS NEEDED */

      // Publish the processed point cloud
      publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}