#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PassThroughFilter {
public:
    float x_min = -10.0; 
    float x_max = 10.0;
    float y_min = -10.0;
    float y_max = 10.0;
    float z_min = -4.0;
    float z_max = 4.5;

    pcl::PointCloud<pcl::PointXYZI>::Ptr  apply(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr  filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> pass_filter;
        //LiDAR area limmit
        //x axis
        pass_filter.setInputCloud(cloud);
        pass_filter.setFilterFieldName("x");
        pass_filter.setFilterLimits(x_min,x_max);
        pass_filter.filter(*filtered_cloud);

        // y axis
        pass_filter.setInputCloud(filtered_cloud);
        pass_filter.setFilterFieldName("y");
        pass_filter.setFilterLimits(y_min,y_max);
        pass_filter.filter(*filtered_cloud);

        // z axis
        pass_filter.setInputCloud(filtered_cloud);
        pass_filter.setFilterFieldName("z");
        pass_filter.setFilterLimits(z_min,z_max);
        pass_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
