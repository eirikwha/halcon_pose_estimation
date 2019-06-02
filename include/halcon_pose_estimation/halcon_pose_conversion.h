//
// Created by eirik on 04.03.19.
//
# pragma once
#ifndef POSE_ESTIMATOR_HALCONPOSECONVERSION_H
#define POSE_ESTIMATOR_HALCONPOSECONVERSION_H

#include <halconcpp/HalconCpp.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <kuka_eki_msgs/KrlPos.h>

using namespace HalconCpp;
using namespace std;

namespace HalconPoseConversion {

    void poseToHomMat(HTuple &pose, HTuple &homMat3D);

    void poseRotToQuat(HTuple &pose, HTuple &dualQuat);

    void poseTrans(HTuple &pose, HTuple &trans);

    void poseToEigen4fPose(HTuple pose, Eigen::Matrix4f &t1);

    void poseToGeometryMsgTuple(HTuple &pose, HTuple &geometryMsg);

    void poseToGeometryMsgPoseStamped(HTuple &pose, geometry_msgs::PoseStamped &poseOut, std::string &frameID);

    void poseToGeometryMsgPose(HTuple &pose, geometry_msgs::Pose &poseOut);

    void poseToKukaXYZRPY(HTuple &pose, kuka_eki_msgs::KrlPos &poseOut);

}

#endif //HALCONMATCHING_HALCONPOSECONVERSION_H
