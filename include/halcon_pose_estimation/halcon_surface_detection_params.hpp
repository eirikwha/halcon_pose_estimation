//
// Created by eirik on 05.06.19.
//

#ifndef HAND_EYE_CALIBRATION_HALCON_SURFACE_DETECTION_PARAMS_HPP
#define HAND_EYE_CALIBRATION_HALCON_SURFACE_DETECTION_PARAMS_HPP

#include <halconcpp/HalconCpp.h>

using namespace HalconCpp;

//todo class implementation for parameter restriction

//* SurfaceModelDetectionParameters
/**
* Model parameters for creation of a 2D Surface Model
*
*/

struct SurfaceDetectionParams {

    SurfaceDetectionParams() :
        relSamplingDistance(HTuple(0.03)),
        returnResultHandle(HTuple("true")),
        keyPointFraction(HTuple(0.5)),
        minScore(HTuple(0)),
        num_matches(HTuple(1)),
        max_overlap_dist_rel(HTuple(0.5)),
        sparse_pose_refinement(HTuple("true")),
        score_type(HTuple("model_point_fraction")),
        pose_ref_use_scene_normals(HTuple("false")),
        dense_pose_refinement(HTuple("true")),
        pose_ref_num_steps(HTuple(5)),
        pose_ref_sub_sampling(HTuple(2)),
        pose_ref_dist_threshold_rel(HTuple(0.1)),
        pose_ref_scoring_dist_rel(HTuple(0.005))
        {}

    HTuple objectModel3D;
    HTuple relSamplingDistance;
    HTuple keyPointFraction;
    HTuple minScore;
    HTuple returnResultHandle;
    HTuple num_matches;
    HTuple max_overlap_dist_rel;
    HTuple sparse_pose_refinement;
    HTuple score_type;
    HTuple pose_ref_use_scene_normals;
    HTuple dense_pose_refinement;
    HTuple pose_ref_num_steps;
    HTuple pose_ref_sub_sampling;
    HTuple pose_ref_dist_threshold_rel;
    HTuple pose_ref_scoring_dist_rel;



};

#endif //HAND_EYE_CALIBRATION_HALCON_SURFACE_DETECTION_PARAMS_HPP
