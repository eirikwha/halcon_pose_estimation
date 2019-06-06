//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_PCLVISUALIZATION_H
#define POSE_ESTIMATOR_PCLVISUALIZATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace PCLViz {

    pcl::visualization::PCLVisualizer simpleVisXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    pcl::visualization::PCLVisualizer simpleVisXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

    pcl::visualization::PCLVisualizer twoViewportsVis(
            pcl::PolygonMesh::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2);

    /*pcl::visualization::PCLVisualizer twoInOneVis(
            pcl::PolygonMesh::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2);
    */

    pcl::visualization::PCLVisualizer twoInOneVis(
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2);

}

#endif //HALCONMATCHING_PCLVISUALIZATION_H
