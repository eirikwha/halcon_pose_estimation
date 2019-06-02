//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_HALCONIO_H
#define POSE_ESTIMATOR_HALCONIO_H

#include "halconcpp/HalconCpp.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace HalconCpp;
using namespace std;

namespace HalconIO {

    HTuple readObjectModel3D(HTuple loadPath, float unit, HTuple &objectModel3D);

    HTuple readObjectModel3DWidthMap(HTuple loadPath, float unit, int cloudWidth, HTuple &objectModel3D);

    HTuple readSurfaceModel(HTuple loadPath, HTuple &surfaceModelID);

    void writeObjectModel3DPly(HTuple outputFileName, HTuple &objectModel3D);

    void writeSurfaceModel(HTuple outputFileName, HTuple &objectModel3D);

    void createModelFromPoints(const HTuple &x, const HTuple &y, const HTuple &z, HTuple &objectModel3D);

    void createModelFromPCLPointXYZ(HTuple &x, HTuple &y, HTuple &z, HTuple &objectModel3D,
                                    pcl::PointCloud<pcl::PointXYZ> cloud);

    void readGenParams(const char *filename, HTuple &genParamName, HTuple &genParamValue);

    void readGenParamValues(const char *filename, HTuple &genParamValue);

    void readYamlConfig(const char *filePath, HTuple &genParamName, HTuple &genParamValue);

}

#endif //HALCONMATCHING_HALCONIO_H
