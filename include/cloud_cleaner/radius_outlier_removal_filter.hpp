#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

class RadiusOutlierRemovalFilter {
public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr apply(pcl::pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;

        radius_filter.setInputCloud(cloud);
        radius_filter.setRadiusSearch(0.8);        // search radius
        radius_filter.setMinNeighborsInRadius(2);  // minimum point clouds num
        radius_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
