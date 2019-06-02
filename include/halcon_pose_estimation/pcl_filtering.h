//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_FILTERING_H
#define POSE_ESTIMATOR_FILTERING_H

#include <pcl/io/pcd_io.h>

namespace PCLFiltering {

    void VoxelGrid(pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr sampledCloud);

    void RemoveNan(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloudFiltered);

    void CropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ> &cloudFiltered,
                    float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
}

#endif //HALCONMATCHING_DOWNSAMPLING_H
