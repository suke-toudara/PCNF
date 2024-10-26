#include <cloud_cleaner/cloud_cleaner.hpp>

namespace cloud_cleaner
{
PointCloudFilterNode::PointCloudFilterNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_filter_node", options) 
{
    declare_parameter<std::vector<std::string>>("filter_types", std::vector<std::string>{"voxel_grid"});
    get_parameter("filter_types", filter_types_);
    
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_points", 10, std::bind(&PointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
}

void PointCloudFilterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = cloud;  // 同じ型に変更

    for (const auto& filter_type : filter_types_) {
        if (filter_type == "voxel_grid") {
            filtered_cloud = voxel_filter_.apply(filtered_cloud);
        } else if (filter_type == "statistical_outlier_removal") {
            filtered_cloud = sor_filter_.apply(filtered_cloud);
        } else if (filter_type == "pass_through") {
            filtered_cloud = pass_filter_.apply(filtered_cloud);
        } else if (filter_type == "radius_outlier_removal") {
            filtered_cloud = radius_filter_.apply(filtered_cloud);
        }
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = msg->header;
    point_cloud_pub_->publish(output_msg);
}
} // namespace cloud_cleaner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cloud_cleaner::PointCloudFilterNode)


