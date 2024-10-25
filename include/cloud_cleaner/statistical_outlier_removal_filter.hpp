#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class StatisticalOutlierRemovalFilter {
public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr apply(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;

        sor_filter.setInputCloud(cloud);
        sor_filter.setMeanK(50);  // 最近傍点の数
        sor_filter.setStddevMulThresh(1.0);  // 標準偏差のしきい値
        sor_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
