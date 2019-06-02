#include <iostream>
#include "Eigen/Core"
#include "halconcpp/HalconCpp.h"
#include "halcon_pose_estimation/halcon_io.h"
#include "halcon_pose_estimation/halcon_surface_matching.h"
#include "halcon_pose_estimation/halcon_surface_model.h"
#include "halcon_pose_estimation/pose_result_handler.h"
#include "halcon_pose_estimation/halcon_pose_conversion.h"
#include "halcon_pose_estimation/halcon_object_model.h"
#include "halcon_pose_estimation/pcl_viz.h"
#include "halcon_pose_estimation/pcl_file_handler.h"

int main() {


    ClearAllObjectModel3d(); // clear from memory
    ClearAllSurfaceModels(); // clear from memory

    HTuple scene;
    HTuple model;
    HTuple genParamName, genParamValue;
    HTuple matchingResultID;
    HTuple poses;
    HTuple bestPose;
    HTuple transformedModel;
    HTuple status;

    const char* surfModelPath = "/home/eirik/catkin_ws/src/halcon_pose_estimation/data/output/MG1_SURFMODEL.sfm";
    const char* objModelPath = "/home/eirik/catkin_ws/src/halcon_pose_estimation/data/input/smallbin_1_ordered.ply";
    const char* stlModelPath = "/home/eirik/catkin_ws/src/halcon_pose_estimation/data/input/MG1.stl";
    const char* transformedStlPath = "/home/eirik/catkin_ws/src/halcon_pose_estimation/data/output/transformed.stl";

    cout << "----------------------------------------\n"
         << "------ FETCHING MODEL AND SCENE: -------\n"
         << "----------------------------------------\n"<< endl;

    HalconIO::readObjectModel3D(objModelPath, 1, scene);
    HalconIO::readSurfaceModel(surfModelPath, model);

    cout << "----------------------------------------\n"
         << "---------- FIND SURFACE MODEL: ---------\n"
         << "----------------------------------------\n"<< endl;

    genParamName.Append("num_matches");
    genParamName.Append("max_overlap_dist_rel");
    genParamName.Append("scene_normal_computation");
    genParamName.Append("sparse_pose_refinement");
    genParamName.Append("score_type");
    genParamName.Append("pose_ref_use_scene_normals");
    genParamName.Append("dense_pose_refinement");
    genParamName.Append("pose_ref_num_steps");
    genParamName.Append("pose_ref_sub_sampling");
    genParamName.Append("pose_ref_dist_threshold_rel");
    genParamName.Append("pose_ref_scoring_dist_rel");

    genParamValue.Append(5);
    genParamValue.Append(0.5);
    genParamValue.Append("mls");
    genParamValue.Append("true");
    genParamValue.Append("model_point_fraction");
    genParamValue.Append("true");
    genParamValue.Append("true");
    genParamValue.Append(10);
    genParamValue.Append(2);
    genParamValue.Append(0.1);
    genParamValue.Append(0.005);


    cout << "Trying to match model and scene." << endl;
    HalconSurfaceMatching::findSurfaceModel3D(model,scene, poses, matchingResultID,
            genParamName, genParamValue);

    PoseResultHandler::printPosesAndScores(matchingResultID,5);

    PoseResultHandler::getSinglePose(matchingResultID,1, bestPose);
    cout << "\nBest pose: " << bestPose.ToString() << endl;

    HTuple rotQuat, trans, geometryMsg;

    HalconPoseConversion::poseTrans(bestPose,trans);
    cout << "Translation: " << trans.ToString() << endl;

    HalconPoseConversion::poseRotToQuat(bestPose, rotQuat);
    cout << "Rotation quaternion: " << rotQuat.ToString() << endl;

    HalconPoseConversion::poseToGeometryMsgTuple(bestPose,geometryMsg);
    cout << "Geometry_msgs format: " << geometryMsg.ToString() << endl;

    Eigen::Matrix4f t;
    HalconPoseConversion::poseToEigen4fPose(bestPose,t);
    cout << "\nHomogenous transformation matrix of the best pose: \n\n" << t << endl << endl;


    cout << "----------------------------------------\n"
         << "------------ TRANSFORMATION: -----------\n"
         << "----------------------------------------\n"<< endl;

    HTuple modelVis;
    HalconIO::readObjectModel3D(stlModelPath, 1, modelVis);

    cout << "Trying to transform model into best pose for visualization" << endl;
    HalconObjectModel::rigidTrans(bestPose, modelVis, transformedModel);

    cout << "Trying to save transformed model." << endl;
    HalconIO::writeObjectModel3DPly(transformedStlPath, transformedModel);

    pcl::PolygonMesh::Ptr model1 (new pcl::PolygonMesh());
    model1 = PCLFileHandler::loadStlToPclMesh(transformedStlPath);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene1 (new pcl::PointCloud<pcl::PointXYZRGB>());
    scene1 = PCLFileHandler::loadPlyToPointXYZRGB(objModelPath);
    PCLViz::twoInOneVis(model1,scene1);

    ClearAllObjectModel3d(); // clear from memory
    ClearAllSurfaceModels(); // clear from memory
    ClearSurfaceMatchingResult(matchingResultID);


}