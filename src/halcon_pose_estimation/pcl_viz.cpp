//
// Created by eirik on 04.03.19.
//

#include "halcon_pose_estimation/pcl_viz.h"

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace PCLViz {

    pcl::visualization::PCLVisualizer simpleVisXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {

        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.initCameraParameters();

        int v1(0);
        viewer.addPointCloud(cloud, "sample cloud 1", v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addText("Scene", 30, 30, 20, 100.0, 100.0, 0.0, "v2 text", v1);
        viewer.addCoordinateSystem(100, "sample cloud 1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 1");
        viewer.resetCamera();
        viewer.spinOnce();

        while (!viewer.wasStopped()) {
            viewer.spin();
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        return (viewer);
    }

    pcl::visualization::PCLVisualizer simpleVisXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {

        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.initCameraParameters();

        int v1(0);
        viewer.addPointCloud(cloud, "sample cloud 1", v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addText("Scene", 30, 30, 20, 100.0, 100.0, 0.0, "v2 text", v1);
        viewer.addCoordinateSystem(100, "sample cloud 1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 1");
        viewer.resetCamera();
        viewer.spinOnce();

        while (!viewer.wasStopped()) {
            viewer.spin();
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        return (viewer);
    }

    pcl::visualization::PCLVisualizer twoViewportsVis(
            pcl::PolygonMesh::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2) {

        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.initCameraParameters();

        // Mesh in v1 viewport
        int v1(0);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, v1);
        viewer.addText("Model", 30, 30, 20, 100.0, 100.0, 0.0, "v1 text", v1);
        viewer.createViewPortCamera(v1);

        viewer.addPolygonMesh(*cloud1, "sample cloud1", v1);

        viewer.addCoordinateSystem(100, "sample cloud1", v1);
        viewer.setCameraPosition(0.0, 0.0, 0.8, 0.0, 0.0, 0.0, v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");

        // Pointcloud in v2 viewport
        int v2(0);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(0, 0, 0, v2);
        viewer.addText("Scene", 30, 30, 20, 100.0, 100.0, 0.0, "v2 text", v2);
        viewer.createViewPortCamera(v2);

        viewer.addPolygonMesh(*cloud1, "sample cloud3", v2); //test
        viewer.addPointCloud(cloud2, "sample cloud2", v2);

        viewer.addCoordinateSystem(100, "sample cloud2", v2);
        viewer.setCameraPosition(0.0, 0.0, -0.01, 0.0, 0.0, 0.0,
                                 v2); // flip directions to match camera frame 0.0,0.0,-0.1,0.0,-1.0,0.0,v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");

        viewer.resetCamera();
        viewer.spinOnce();

        while (!viewer.wasStopped()) {
            viewer.spin();
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        return (viewer);
    }

    pcl::visualization::PCLVisualizer twoInOneVis(
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2) {

        /*pcl::PolygonMesh::ConstPtr cloud1*/

        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.initCameraParameters();

        int v1(0);
        //viewer.addPolygonMesh(*cloud1, "mesh 1", v1); //test
        viewer.addPointCloud(cloud1, "sample cloud 2", v1);
        viewer.addPointCloud(cloud2, "sample cloud 1", v1);

        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addText("Scene", 30, 30, 20, 100.0, 100.0, 0.0, "v2 text", v1);
        viewer.addCoordinateSystem(100, "sample cloud 1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.8, "sample cloud 1");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud 2");
        viewer.resetCamera();
        viewer.spinOnce();

        while (!viewer.wasStopped()) {
            viewer.spin();
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        return (viewer);
    }
}