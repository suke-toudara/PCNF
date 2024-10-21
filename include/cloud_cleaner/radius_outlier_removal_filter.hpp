#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

class RadiusOutlierRemovalFilter {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;

        radius_filter.setInputCloud(cloud);
        radius_filter.setRadiusSearch(0.8);  // 検索半径 (単位: メートル)
        radius_filter.setMinNeighborsInRadius(2);  // 半径内に必要な最小近傍点数
        radius_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
