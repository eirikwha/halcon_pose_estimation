//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_HOBJECTMODEL3DUTILS_H
#define POSE_ESTIMATOR_HOBJECTMODEL3DUTILS_H

#include <halconcpp/HalconCpp.h>

using namespace HalconCpp;
using namespace std;

namespace HalconObjectModel {

    bool hasPoints(HTuple &objectModel3D);

    bool hasPointNormals(HTuple &objectModel3D);

    bool hasSurfaceBasedMatchingData(HTuple &objectModel3D);

    int getNumberOfPoints(HTuple &objectModel3D);

    int getNumberOfTriangles(HTuple &objectModel3D);

    int getNumberOfFaces(HTuple &objectModel3D);

    void getBoundingBox(HTuple objectModel3D, double &min_x, double &min_y, double &min_z, double &max_x, double &max_y,
                        double &max_z);

    HTuple getDiameter(HTuple &objectModel3D);

    void computeSmallestBoundingBox(HTuple &objectModel3D, HTuple Type, HTuple &Pose, HTuple &Length1, HTuple &Length2,
                                    HTuple &Length3);

    void computeSmallestBoundingSphere(HTuple &objectModel3D, HTuple &CenterPoint_out, HTuple &Radius_out);

    HTuple computeMaxDiameter(HTuple &objectModel3D);

    void computeArea(HTuple &objectModel3D, HTuple &areaOut);

    HTuple computePrincipalAxis(HTuple &objectModel3D);

    HTuple computeMeanPoint(HTuple &objectModel3D);

    void computeConvexHullOfObjectModel(HTuple &objectModel3D, HTuple &convexHullObjectModelOut);

    void computeSurfaceNormals(HTuple &objectModel3D);

    void sampleObjectModel(HTuple SampleDistance, HTuple &objectModel3D);

    void affineTrans(HTuple &TransformationPose, HTuple &objectModelIn, HTuple &transformedModelOut);

    void rigidTrans(HTuple &TransformationPose, HTuple &objectModelIn, HTuple &TransformedModel_out);

    void showModels(HTuple &objectModel3D);

}

#endif //HALCONMATCHING_HOBJECTMODEL3DUTILS_H
