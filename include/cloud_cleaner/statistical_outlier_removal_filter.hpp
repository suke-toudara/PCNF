#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class StatisticalOutlierRemovalFilter {
public:
    int search_pc_num = 50;
    float std_thresh = 1.0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr apply(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;

        sor_filter.setInputCloud(cloud);
        sor_filter.setMeanK(search_pc_num);
        sor_filter.setStddevMulThresh(std_thresh); 
        sor_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};

