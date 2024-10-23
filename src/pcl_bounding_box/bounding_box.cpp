#include "bounding_box.hpp"

// Helper function to create the corners of the cube
std::vector<geometry_msgs::msg::Point> BoundingBox::createCubeCorners(const Eigen::Vector4f &min_point, const Eigen::Vector4f &max_point)
{
    std::vector<geometry_msgs::msg::Point> corners(8);

    corners[0].x = min_point.x();
    corners[0].y = min_point.y();
    corners[0].z = min_point.z();

    corners[1].x = max_point.x();
    corners[1].y = min_point.y();
    corners[1].z = min_point.z();

    corners[2].x = max_point.x();
    corners[2].y = max_point.y();
    corners[2].z = min_point.z();

    corners[3].x = min_point.x();
    corners[3].y = max_point.y();
    corners[3].z = min_point.z();

    corners[4].x = min_point.x();
    corners[4].y = min_point.y();
    corners[4].z = max_point.z();

    corners[5].x = max_point.x();
    corners[5].y = min_point.y();
    corners[5].z = max_point.z();

    corners[6].x = max_point.x();
    corners[6].y = max_point.y();
    corners[6].z = max_point.z();

    corners[7].x = min_point.x();
    corners[7].y = max_point.y();
    corners[7].z = max_point.z();

    return corners;
}

// Create a bounding box marker using LINE_LIST
visualization_msgs::msg::Marker BoundingBox::createBoundingBoxMarker(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster,
    const std_msgs::msg::Header &header,
    int id)
{
    // Compute the bounding box for the cluster
    Eigen::Vector4f min_point, max_point;
    pcl::getMinMax3D(*cluster, min_point, max_point);

    // Create a marker
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "bounding_box";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02; // Line width

    // Set marker color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Get the corners of the bounding box
    std::vector<geometry_msgs::msg::Point> corners = createCubeCorners(min_point, max_point);

    // Add line segments (12 lines to form a cube)
    std::vector<std::pair<int, int>> edges = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Bottom square
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Top square
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Connecting edges
    };

    for (const auto &edge : edges)
    {
        marker.points.push_back(corners[edge.first]);
        marker.points.push_back(corners[edge.second]);
    }

    return marker;
}
