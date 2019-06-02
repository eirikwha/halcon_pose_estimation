//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_HALCONSURFACEDETECTOR_H
#define POSE_ESTIMATOR_HALCONSURFACEDETECTOR_H

#include "halconcpp/HalconCpp.h"
using namespace HalconCpp;
using namespace std;

namespace HalconSurfaceMatching {

    void findSurfaceModel3D(const HTuple &model, const HTuple &scene, HTuple &poses, HTuple &matchingResultID,
                            HTuple &genParamName, HTuple &genParamValue);

    void findSurfaceModel3DEdges(const HTuple &model, const HTuple &scene, const HObject &sceneImage, HTuple &poses,
                                 HTuple &matchingResultID, HTuple &genParamName, HTuple &genParamValue);
}

#endif //HALCONMATCHING_HALCONSURFACEDETECTOR_H
