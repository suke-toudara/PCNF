#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_map>
#include <Eigen/Core>

// Eigen::Vector3i 用のハッシュ関数を作成
struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i &key) const {
        return std::hash<int>()(key.x()) ^ std::hash<int>()(key.y()) ^ std::hash<int>()(key.z());
    }
};

class VoxelGridFilter {
public:
    float voxel_size = 0.1f; 
    int min_points = 4; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr apply(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        return filterByOccupancy(cloud);
    }

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterByOccupancy(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
        std::unordered_map<Eigen::Vector3i, int, Vector3iHash> occupancy_map;
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 各点についてボクセルインデックスを計算
        for (const auto &point : cloud->points) {
            Eigen::Vector3i voxel_index(static_cast<int>(point.x / voxel_size),
                                         static_cast<int>(point.y / voxel_size),
                                         static_cast<int>(point.z / voxel_size));
            occupancy_map[voxel_index]++;
        }

        // 条件を満たすボクセルの点をフィルタリング
        for (const auto &point : cloud->points) {
            Eigen::Vector3i voxel_index(static_cast<int>(point.x / voxel_size),
                                         static_cast<int>(point.y / voxel_size),
                                         static_cast<int>(point.z / voxel_size));
            if (occupancy_map[voxel_index] >= min_points) {
                filtered_cloud->points.push_back(point);
            }
        }
        filtered_cloud->width = static_cast<uint32_t>(filtered_cloud->points.size());
        filtered_cloud->height = 1; 
        filtered_cloud->is_dense = false; 
        return filtered_cloud;
    }
};
