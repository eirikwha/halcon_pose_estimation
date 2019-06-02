//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_SURFACEMODELUTILS_H
#define POSE_ESTIMATOR_SURFACEMODELUTILS_H

#include "halconcpp/HalconCpp.h"

using namespace HalconCpp;
using namespace std;

namespace HalconSurfaceModel {

    void createHalconSurfaceModel(HTuple filename);

    bool hasTrainedEdges(HTuple &surfaceModel);

    void createSurfaceModel(HTuple &objectModel3D, float relSampDist,
                            HTuple &genParamName, HTuple &genParamValue, HTuple &surfaceModel);

    void getSampledModel(HTuple &surfaceModel, HTuple &sampledObjectModelOut);

    void getCenterOfSurfaceModel(HTuple &surfaceModel, HTuple &centerOut);

    void getBoundingBoxSurfaceModel(HTuple &surfaceModel, HTuple &boundingBoxOut);

    HTuple getDiameterSurfaceModel(HTuple &surfaceModel);

    void showSurfaceModel(HTuple &surfaceModel);

}

#endif //HALCONMATCHING_SURFACEMODELUTILS_H
