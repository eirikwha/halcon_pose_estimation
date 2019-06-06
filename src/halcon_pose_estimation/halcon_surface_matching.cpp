//
// Created by eirik on 04.03.19.
//

#include "halcon_pose_estimation/halcon_surface_matching.h"
#include "halcon_pose_estimation/halcon_surface_detection_params.hpp"

namespace HalconSurfaceMatching{

    void findSurfaceModel3D(const HTuple &model, const HTuple &scene, HTuple &poses,
                            HTuple &matchingResultID, HTuple &genParamName, HTuple &genParamValue){

        HTuple start1, end1, time1;

        // TODO: Get these settings in config file as well
        // TODO: Multiple calib1_pose. Make selectable in config file?? Or maybe in separate setup. HTML/JS/QT GREAT for this

        const HTuple sceneRelSamplingDistance(0.03);
        const HTuple sceneKeyPointFraction(0.3);
        const HTuple minScore(0.2);
        const HTuple result("true");
        HTuple score;

        try {
            FindSurfaceModel(model,scene,sceneRelSamplingDistance,sceneKeyPointFraction,minScore,result, genParamName, genParamValue,&poses,&score,&matchingResultID);
            cout << "Successful" << endl;
        }
        catch(HException &exc){
            cout << exc.ErrorMessage() << endl;
        }

        ClearAllObjectModel3d(); // clear from memory
        ClearAllSurfaceMatchingResults(); // clear from memory
    }

    void findSurfaceModel3DEdges(const HTuple &model, const HTuple &scene, const HObject &sceneImage, HTuple &poses,
                                 HTuple &matchingResultID, HTuple &genParamName, HTuple &genParamValue){

        HTuple start1, end1, time1;

        const HTuple sceneRelSamplingDistance(0.03);
        const HTuple sceneKeyPointFraction(0.3);
        const HTuple minScore(0.2);
        const HTuple result("true");
        HTuple score;

        try {
            FindSurfaceModelImage(sceneImage, model,scene,sceneRelSamplingDistance,sceneKeyPointFraction,minScore,result, genParamName, genParamValue,&poses,&score,&matchingResultID);
        }
        catch(HException &exc){
            cout << exc.ErrorMessage() << endl;
        }

        ClearAllObjectModel3d(); // clear from memory
        ClearAllSurfaceMatchingResults(); // clear from memory

    }
/*
    void setParameters(const SurfaceModelDetectionParameters& param)
    {
        mParam = param;
    }
    */
}
