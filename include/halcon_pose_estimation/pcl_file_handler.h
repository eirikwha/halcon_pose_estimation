//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_PCLFILEHANDLER_H
#define POSE_ESTIMATOR_PCLFILEHANDLER_H

#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//using namespace pcl;

namespace PCLFileHandler {

    void showHelp(char *program_name);

    bool loadCloud(const std::string &filename, pcl::PCLPointCloud2 &cloud);

    void saveCloud(const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format);

    vtkSmartPointer<vtkPolyData> loadStlToVtkPoly(const char *filename);

    pcl::PolygonMeshPtr loadStlToPclMesh(const char *filename);

    pcl::PointCloud<pcl::PointXYZ>::Ptr polyMeshToPointCloud(const char *filename);

    void printHelp(int, char **argv);

    pcl::PCLPointCloud2 loadPlyToPCLPointCloud2(const char *filename);

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud2ToPointXYZRGB(pcl::PCLPointCloud2 cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPlyToPointXYZRGB(const char *filename);

    void savePCDPointCloud2 (std::string filename, pcl::PCLPointCloud2 cloud);

    void savePCDPointCloudXYZRGB (std::string filename, pcl::PointCloud<pcl::PointXYZRGB> cloud);

}


#endif //HALCONMATCHING_PCLFILEHANDLER_H
