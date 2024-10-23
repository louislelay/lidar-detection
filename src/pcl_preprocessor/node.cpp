#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "filter.hpp"

using std::placeholders::_1;


class PointCloudPreProcessor : public rclpp::Node
{
  public:
    PointCloudPreProcessor() : Node("pointcloud_preprocessor")
    {
      // Create Subscriber to "input_pcl" topic
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input_pcl",
        10,
        std::bind(&PointCloudPreProcessor::pointcloud_callback, this, _1));

      // Create publisher to "output_pcl" topic
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output_pcl", 
        10);
    }

  private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with %d data points", msg->width * msg->height);

      // Create output message
      sensor_msgs::msg::PointCloud2 output_msg;

      filter.preprocessPointCloud(msg, output_msg);

      // Publish the preprocessed point cloud
      publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    PointCloudFilter filter;  // Filter class object
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPreProcessor>());
    rclcpp::shutdown();
    return 0;
}