#include <cloud_cleaner/cloud_cleaner.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cloud_cleaner::PointCloudFilterNode>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}