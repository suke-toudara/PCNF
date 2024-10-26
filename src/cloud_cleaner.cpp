#include <cloud_cleaner/cloud_cleaner.hpp>

namespace cloud_cleaner
{
PointCloudFilterNode::PointCloudFilterNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_filter_node", options) 
{
    declare_parameter<std::vector<std::string>>("filter_types", std::vector<std::string>{"voxel_grid"});
    get_parameter("filter_types", filter_types_);
    SetFilterParam();
    
    std::string cloud_topic_name;
    std::string filter_cloud_topic_name;
    declare_parameter<std::string>("cloud_topic_name","/lidar_points");
    declare_parameter<std::string>("filter_cloud_topic_name","/filtered_cloud");
    get_parameter("cloud_topic_name", cloud_topic_name);
    get_parameter("filter_cloud_topic_name", filter_cloud_topic_name);

    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_name, 10, std::bind(&PointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(filter_cloud_topic_name, 10);
}

void PointCloudFilterNode::SetFilterParam(){
    for (const auto& filter_type : filter_types_) {
        if (filter_type == "voxel_grid") {
            declare_parameter("voxel_size",0.1);
            declare_parameter("min_points",5);
            get_parameter("voxel_size", voxel_filter_.voxel_size);
            get_parameter("min_points", voxel_filter_.min_points);

        } else if (filter_type == "statistical_outlier_removal") {
            declare_parameter("search_pc_num",50);
            declare_parameter("std_thresh",1.0);
            get_parameter("search_radius", sor_filter_.search_pc_num);
            get_parameter("min_pc_num", sor_filter_.std_thresh);

        } else if (filter_type == "pass_through") {
            declare_parameter("x_min",-10.0);
            declare_parameter("x_max", 10.0);
            declare_parameter("y_min",-5.0);
            declare_parameter("y_max", 5.0);
            declare_parameter("z_min", 0.0);
            declare_parameter("z_max", 3.5);
            get_parameter("x_min", pass_filter_.x_min);
            get_parameter("x_max", pass_filter_.x_max);
            get_parameter("y_min", pass_filter_.y_min);
            get_parameter("y_max", pass_filter_.y_max);
            get_parameter("z_min", pass_filter_.z_min);            
            get_parameter("z_max", pass_filter_.z_max);
        
        } else if (filter_type == "radius_outlier_removal") {
            declare_parameter("search_radius",0.3);
            declare_parameter("min_pc_num",5);
            get_parameter("search_radius", radius_filter_.search_radius);
            get_parameter("min_pc_num", radius_filter_.min_pc_num);
        }
    }

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


