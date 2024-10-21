#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class PassThroughFilter {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass_filter;

        pass_filter.setInputCloud(cloud);
        pass_filter.setFilterFieldName("z");  // z軸に沿ったフィルタリング
        pass_filter.setFilterLimits(0.0, 1.5);  // z軸方向のフィルタリング範囲 (0～1.5メートル)
        pass_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
