#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cloud_cleaner/voxel_grid_filter.hpp"
#include "cloud_cleaner/statistical_outlier_removal_filter.hpp"
#include "cloud_cleaner/pass_through_filter.hpp"
#include "cloud_cleaner/radius_outlier_removal_filter.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

namespace cloud_cleaner
{
class PointCloudFilterNode : public rclcpp::Node 
{
public:
    explicit PointCloudFilterNode(const rclcpp::NodeOptions & options);

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    VoxelGridFilter voxel_filter_;
    StatisticalOutlierRemovalFilter sor_filter_;
    PassThroughFilter pass_filter_;
    RadiusOutlierRemovalFilter radius_filter_;

    std::vector<std::string> filter_types_;  // フィルターのリスト
};
}