#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class VoxelGridFilter {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // ボクセルの大きさ (単位: メートル)
        voxel_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
};
