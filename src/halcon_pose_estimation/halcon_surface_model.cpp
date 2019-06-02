//
// Created by eirik on 04.03.19.
//

#include "halcon_pose_estimation/halcon_surface_model.h"

// TODO: Split up and clean!!!

namespace HalconSurfaceModel {

    void createSurfaceModel(HTuple &objectModel3D, float relSampDist,
                            HTuple &genParamName, HTuple &genParamValue, HTuple &surfaceModel) {

        try {
            CreateSurfaceModel(objectModel3D, relSampDist, genParamName, genParamValue, &surfaceModel);
            cout << "Model creation sucessful." << endl;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void getSampledModel(HTuple &surfaceModel, HTuple &sampledObjectModelOut) {

        try {
            GetSurfaceModelParam(surfaceModel, "sampled_model", &sampledObjectModelOut);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void getCenterOfSurfaceModel(HTuple &surfaceModel, HTuple &centerOut) {

        try {
            GetSurfaceModelParam(surfaceModel, "center", &centerOut);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void getBoundingBoxSurfaceModel(HTuple &surfaceModel, HTuple &boundingBoxOut) {

        try {
            GetSurfaceModelParam(surfaceModel, "bounding_box1", &boundingBoxOut);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    HTuple getDiameterSurfaceModel(HTuple &surfaceModel) {
        HTuple diaOut;
        try {
            GetSurfaceModelParam(surfaceModel, "diameter", &diaOut);
            return diaOut;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    bool hasTrainedEdges(HTuple &surfaceModel) {
        HTuple has3dEdges;
        try {
            GetSurfaceModelParam(surfaceModel, "3d_edges_trained", &has3dEdges);
            if (has3dEdges == "true") {
                return true;
            } else {
                return false;
            }
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void showSurfaceModel(HTuple &surfaceModel) { // NOT WORKING PROPERLY, USING PCL/RVIZ FOR NOW
        try {
            HTuple width = 1000;
            HTuple height = 800;

            HWindow w(0, 0, (Hlong) width, (Hlong) height);

            DispObjectModel3d(w, surfaceModel, HTuple(), HTuple(), HTuple("colored"), HTuple(3));

            w.Click();
            w.CloseWindow();
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }
}
