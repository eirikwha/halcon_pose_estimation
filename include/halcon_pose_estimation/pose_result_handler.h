//
// Created by eirik on 05.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_POSERESULTHANDLER_H
#define POSE_ESTIMATOR_POSERESULTHANDLER_H

#include "halconcpp/HalconCpp.h"
using namespace HalconCpp;
using namespace std;

namespace PoseResultHandler {

    int getNumMatches(HTuple &matchPoses);

    void getSinglePose(HTuple &matchingResultID, int resultNumber, HTuple &singlePoseOut);

    void getKeyPoints(HTuple &matchingResultID, int resultNumber, HTuple &keyPointsOut);

    void getSampledEdges(HTuple &matchingResultID, int resultNumber, HTuple &sampledEdgesOut);

    void getSampledScene(HTuple &matchingResultID, int resultNumber, HTuple &sampledSceneOut);

    void printPoses(HTuple &matchingResultID, int numMatches);

    void printScoreUnrefined(HTuple &matchingResultID, int numMatches);

    void printScoreRefined(HTuple &matchingResultID, int numMatches);

    void printPosesAndScores(HTuple &matchingResultID, int numMatches);

}

#endif //HALCONMATCHING_POSERESULTHANDLER_H
