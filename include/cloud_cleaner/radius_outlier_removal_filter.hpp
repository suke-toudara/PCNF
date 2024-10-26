#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

class RadiusOutlierRemovalFilter {
public:
    float search_radius = 0.8;
    int min_pc_num = 2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr apply(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;

        radius_filter.setInputCloud(cloud);
        radius_filter.setRadiusSearch(search_radius);        // search radius
        radius_filter.setMinNeighborsInRadius(min_pc_num);  // minimum point clouds num
        radius_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
