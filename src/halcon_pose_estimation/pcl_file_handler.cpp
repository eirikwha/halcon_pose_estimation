//
// Created by eirik on 04.03.19.
//

#include "halcon_pose_estimation/pcl_file_handler.h"

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/console/print.h>
#include <pcl/console/time.h>

//#include <pcl/conversions.h>

using namespace std;
using namespace pcl::console;

namespace PCLFileHandler {

    void showHelp(char *program_name) {
        cout << endl;
        cout << "ERROR: Wrong file format. Usage: " << program_name << " cloud_filename.[ply]" << endl;
        cout << "-h:  Show this help." << endl;
    }

// TODO: Get print highlights and loading data for the old functions.
// TODO: Consistent pointer/reference input/output. Too messy!!
// TODO: Clean up argv, argc arguments. Too messy!!


    vtkSmartPointer<vtkPolyData> loadStlToVtkPoly(const char *filename) {

        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(filename);
        reader->Update();
        vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();

        return polyData;

    }

    pcl::PolygonMeshPtr loadStlToPclMesh(const char *filename) {

        vtkSmartPointer<vtkPolyData> polyData = loadStlToVtkPoly(filename);
        pcl::PolygonMeshPtr stlPolyMesh(new pcl::PolygonMesh());
        pcl::io::vtk2mesh(polyData, *stlPolyMesh);

        return stlPolyMesh;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr polyMeshToPointCloud(const char *filename) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
        vtkSmartPointer<vtkPolyData> polyData = loadStlToVtkPoly(filename);
        pcl::io::vtkPolyDataToPointCloud(polyData, *outputCloud);

        return outputCloud;
    }

    bool loadCloud(const std::string &filename, pcl::PCLPointCloud2 &cloud) {
        TicToc tt;
        print_highlight("Loading ");
        print_value("%s ", filename.c_str());

        pcl::PLYReader reader;
        tt.tic();
        if (reader.read(filename, cloud) < 0)
            return (false);
        print_info("[done, ");
        print_value("%g", tt.toc());
        print_info(" ms : ");
        print_value("%d", cloud.width * cloud.height);
        print_info(" points]\n");
        print_info("Available dimensions: ");
        print_value("%s\n", pcl::getFieldsList(cloud).c_str());

        return (true);
    }


    void saveCloud(const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format) {
        TicToc tt;
        tt.tic();

        print_highlight("Saving ");
        print_value("%s ", filename.c_str());

        pcl::PCDWriter writer;
        writer.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), format);

        print_info("[done, ");
        print_value("%g", tt.toc());
        print_info(" ms : ");
        print_value("%d", cloud.width * cloud.height);
        print_info(" points]\n");
    }

    void printHelp(int, char **argv) {
        print_error("Syntax is: %s input.ply \n", argv[0]);
    }

//TODO: Change argument to std::string filename??

    pcl::PCLPointCloud2 loadPlyToPCLPointCloud2(const char *filename) {

        print_info("Load and convert a PLY file to PCD format. ");

        pcl::PCLPointCloud2 cloud;
        if (!loadCloud(filename, cloud))
            return cloud;
    }


    pcl::PointCloud<pcl::PointXYZRGB> pointCloud2ToPointXYZRGB(pcl::PCLPointCloud2 cloud) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(cloud, *newCloud);
        return *newCloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPlyToPointXYZRGB(const char *filename) {
        pcl::PCLPointCloud2 cloud = loadPlyToPCLPointCloud2(filename);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *newCloud = pointCloud2ToPointXYZRGB(cloud);
        return newCloud;
    }


    void savePCDPointCloud2(std::string filename, pcl::PCLPointCloud2 cloud) {

        TicToc tt;
        tt.tic();

        bool format = true;
        print_highlight("Saving ");
        print_value("%s ", filename.c_str());

        // Convert to PLY and save
        saveCloud(filename, cloud, format);
        print_info("[done, ");
        print_value("%g", tt.toc());
        print_info(" ms : ");
        print_value("%d", cloud.width * cloud.height);
        print_info(" points]\n");
    }


    void savePCDPointCloudXYZRGB(std::string filename, pcl::PointCloud<pcl::PointXYZRGB> cloud) {

        TicToc tt;
        tt.tic();

        print_highlight("Saving ");
        print_value("%s ", filename.c_str());
        pcl::io::savePCDFileASCII(filename, cloud);
        print_info("[done, ");
        print_value("%g", tt.toc());
        print_info(" ms : ");
        print_value("%d", cloud.width * cloud.height);
        print_info(" points]\n");
    }
}
