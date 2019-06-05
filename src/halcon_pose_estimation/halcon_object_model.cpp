//
// Created by eirik on 04.03.19.
//

#include "halcon_pose_estimation/halcon_object_model.h"

namespace HalconObjectModel {

    bool hasPoints(HTuple &objectModel3D) {

        HTuple haspoints = 0;
        bool res = false;
        try {
            GetObjectModel3dParams(objectModel3D, HTuple("has_points"), &haspoints);

            string value = (string) haspoints.ToString();
            if (value.compare("true") == 0) {
                res = true;
            } else
                res = false;

        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in hasPoints()" << endl;
        }
        return res;
    }

    bool hasPointNormals(HTuple &objectModel3D) {

        HTuple hasPointNormals = 0;
        bool res = false;
        try {
            GetObjectModel3dParams(objectModel3D, HTuple("has_point_normals"), &hasPointNormals);

            string value = (string) hasPointNormals.ToString();
            res = value.compare("true") == 0;

        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in hasPointNormals()" << endl;
        }
        return res;
    }

    bool hasSurfaceBasedMatchingData(HTuple &objectModel3D) {

        HTuple hasSurfaceBasedMatchingData = 0;
        bool res = false;
        try {
            GetObjectModel3dParams(objectModel3D, HTuple("has_surface_based_matching_data"),
                                   &hasSurfaceBasedMatchingData);

            string value = (string) hasSurfaceBasedMatchingData.ToString();
            res = value.compare("true") == 0;

        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in hasSurfaceBasedMatchingData()" << endl;
        }
        return res;
    }

    int getNumberOfPoints(HTuple &objectModel3D) {

        HTuple NumberOfPoints = 0;
        int res = 0;
        try {
            GetObjectModel3dParams(objectModel3D, HTuple("num_points"), &NumberOfPoints);

            res = (int) NumberOfPoints[0];

        } catch (HException &except) {
            cout << except.ErrorMessage() << " in getNumberOfPoints()" << endl;
        }
        return res;
    }

    int getNumberOfTriangles(HTuple &objectModel3D) {

        HTuple NumberOfTriangles = 0;
        int res = 0;
        try {
            GetObjectModel3dParams(objectModel3D, HTuple("num_triangles"), &NumberOfTriangles);

            res = (int) NumberOfTriangles[0];

        } catch (HException &except) {
            cout << except.ErrorMessage() << " in getNumberOfTriangles()" << endl;
        }
        return res;
    }

    int getNumberOfFaces(HTuple &objectModel3D) {

        HTuple NumberOfFaces = 0;
        int res = 0;
        try {
            GetObjectModel3dParams(objectModel3D, HTuple("num_faces"), &NumberOfFaces);

            res = (int) NumberOfFaces[0];

        } catch (HException &except) {
            cout << except.ErrorMessage() << " in Create3DObjectModel::getNumberOfFaces()" << endl;
        }
        return res;
    }

    void getBoundingBox(HTuple &objectModel3D, double &min_x, double &min_y, double &min_z, double &max_x, double &max_y,
                   double &max_z) {

        HTuple data = 0;

        try {
            GetObjectModel3dParams(objectModel3D, HTuple("bounding_box1"), &data);

            min_x = (double) data[0];
            min_y = (double) data[1];
            min_z = (double) data[2];
            max_x = (double) data[3];
            max_y = (double) data[4];
            max_z = (double) data[5];

        } catch (HException &except) {
            cout << except.ErrorMessage() << " in Create3DObjectModel::getBoundingBox()" << endl;
        }
    }

    HTuple getDiameter(HTuple &objectModel3D) {

        HTuple diameter;

        try {
            GetObjectModel3dParams(objectModel3D, HTuple("diameter_axis_aligned_bounding_box"), &diameter);
        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in Create3DObjectModel::getDiameter()" << endl;
        }

        return diameter;
    }

    void computeSmallestBoundingBox(HTuple &objectModel3D, HTuple Type, HTuple &Pose, HTuple &Length1, HTuple &Length2,
                                    HTuple &Length3) {

        try {
            SmallestBoundingBoxObjectModel3d(objectModel3D, Type, &Pose, &Length1, &Length2, &Length3);

        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computeSmallestBoundingBox()" << endl;
        }
    }

    void computeSmallestBoundingSphere(HTuple &objectModel3D, HTuple &CenterPoint_out, HTuple &Radius_out) {

        try {
            SmallestSphereObjectModel3d(objectModel3D, &CenterPoint_out, &Radius_out);

        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computeSmallestBoundingSphere()" << endl;
        }
    }

    HTuple computeMaxDiameter(HTuple &objectModel3D) {

        HTuple Diameter_out = 0;
        try {
            MaxDiameterObjectModel3d(objectModel3D, &Diameter_out);


        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computeMaxDiameter()" << endl;
        }

        return Diameter_out;
    }

    void computeArea(HTuple &objectModel3D, HTuple &areaOut) {

        try {
            AreaObjectModel3d(objectModel3D, &areaOut);

        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computeArea()" << endl;
        }
    }

    HTuple computePrincipalAxis(HTuple &objectModel3D) {

        HTuple principalAxis = 0;
        try {
            MomentsObjectModel3d(objectModel3D, HTuple("principal_axis"), &principalAxis);


        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computePrincipalAxis()" << endl;
        }
        return principalAxis;
    }

    HTuple computeMeanPoint(HTuple &objectModel3D) {

        HTuple meanPoint = 0;
        try {
            MomentsObjectModel3d(objectModel3D, HTuple("mean_points"), &meanPoint);

        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computeMeanPoint()" << endl;
        }
        return meanPoint;
    }

    void computeConvexHullOfObjectModel(HTuple &objectModel3D, HTuple &convexHullObjectModelOut) {

        try {
            ConvexHullObjectModel3d(objectModel3D, &convexHullObjectModelOut);
        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computeConvexHullOfObjectModel()" << endl;
        }
    }

    void computeSurfaceNormals(HTuple &objectModel3D) {

        HTuple GenParamName;
        HTuple GenParamValue;
        try {
            SurfaceNormalsObjectModel3d(objectModel3D, HTuple("mls"), GenParamName, GenParamValue, &objectModel3D);
        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in computeSurfaceNormals()" << endl;
        }
    }

    void sampleObjectModel(HTuple SampleDistance, HTuple &objectModel3D) {

        HTuple GenParamName;
        HTuple GenParamValue;
        try {
            SampleObjectModel3d(objectModel3D, HTuple("fast"), SampleDistance, GenParamName, GenParamValue,
                                &objectModel3D);
        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in sampleObjectModel()" << endl;
        }
    }

    void affineTrans(HTuple &TransformationPose, HTuple &objectModelIn, HTuple &transformedModelOut) {

        HTuple HomMat3D;
        try {
            PoseToHomMat3d(TransformationPose, &HomMat3D);
            AffineTransObjectModel3d(objectModelIn, HomMat3D, &transformedModelOut);
        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in affineTransformation()" << endl;
        }
    }

    void rigidTrans(HTuple &TransformationPose, HTuple &objectModelIn, HTuple &transformedModelOut) {

        try {
            RigidTransObjectModel3d(objectModelIn, TransformationPose, &transformedModelOut);
        }
        catch (HException &except) {
            cout << except.ErrorMessage() << " in rigidTransformation()" << endl;
        }
    }

// TODO: Make this work or delete???
    void showModels(HTuple &objectModel3D) {
        try {
            HTuple width = 1000;
            HTuple height = 800;

            HWindow w(0, 0, (Hlong) width, (Hlong) height);

            DispObjectModel3d(w, objectModel3D, HTuple(), HTuple(), HTuple("colored"), HTuple(3));

            w.Click();
            w.CloseWindow();
        }
        catch (HException &except) {
            cout << except.ErrorMessage() << "in showModels()" << endl;
        }
    }
}