//
// Created by eirik on 04.03.19.
//

#include "halcon_pose_estimation/pcl_filtering.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>

namespace PCLFiltering {

    void VoxelGrid(pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr cloudFiltered) {

// Create the filtering object
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloudFiltered);
    }

    void RemoveNan(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloudFiltered) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud, cloudFiltered, indices);
    }

    void CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ> &cloudFiltered,
                    float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {

        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
        boxFilter.setInputCloud(cloud);
        boxFilter.filter(cloudFiltered);
    }
}