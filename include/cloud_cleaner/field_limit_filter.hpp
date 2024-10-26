#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PassThroughFilter {
public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr  apply(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr  filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> pass_filter;
        //LiDAR area limmit
        //x axis
        pass_filter.setInputCloud(cloud);
        pass_filter.setFilterFieldName("x");
        pass_filter.setFilterLimits(-10.0, 10.0);
        pass_filter.filter(*filtered_cloud);

        // y axis
        pass_filter.setInputCloud(filtered_cloud);
        pass_filter.setFilterFieldName("y");
        pass_filter.setFilterLimits(-5.0, 5.0);
        pass_filter.filter(*filtered_cloud);

        // z axis
        pass_filter.setInputCloud(filtered_cloud);
        pass_filter.setFilterFieldName("z");
        pass_filter.setFilterLimits(0.0, 3.5);
        pass_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
