#include <cloud_cleaner/cloud_cleaner.hpp>

namespace cloud_cleaner
{
PointCloudFilterNode::PointCloudFilterNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_filter_node", options) 
{
    // filter_typesをパラメータとして取得
    this->declare_parameter<std::vector<std::string>>("filter_types", std::vector<std::string>{"voxel_grid"});
    this->get_parameter("filter_types", filter_types_);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/input_cloud", 10, std::bind(&PointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
}

void PointCloudFilterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // sensor_msgs::PointCloud2をPCLのPointCloudに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // フィルター処理
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = cloud; // 最初は入力クラウドを設定

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

    // フィルター後のデータをROSメッセージに変換してパブリッシュ
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    point_cloud_pub_->publish(output_msg);
}
} // namespace cloud_cleaner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cloud_cleaner::PointCloudFilterNode)


