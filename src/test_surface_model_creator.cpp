//
// Created by eirik on 12.03.19.
//

// TODO: SET INPUT AND OUTPUT FILENAME, PASS FROM MAIN
// Use parsing
// Accept ply and stl as input
// Set default values
// TODO: Intermediate visualization of normals, keypoints and samples.

#include "halcon_pose_estimation/halcon_io.h"
#include "halcon_pose_estimation/halcon_object_model.h"
#include "halcon_pose_estimation/halcon_surface_model.h"
#include "asr_halcon_bridge/halcon_pointcloud.h"
#include <sensor_msgs/PointCloud2.h>
#include "halcon_pose_estimation/pcl_file_handler.h"
#include <pcl/visualization/pcl_visualizer.h>

void printHelp(int, char **argv){
    cout <<  "Arguments are: 'yourpath'/inputfilename  'yourpath'/outputfilename {default|custom} {standard|edges}" << endl << endl;
    cout <<  "NB! Training a model with edges will take a long time, possibly more than 15 minutes depending on your resources" << endl;
}


int main(int argc, char **argv){

    if (argc < 4 || argv[1] == "--h" || argv[1] == "-h"){
        printHelp(argc,argv);
        return -1;
    }

    float unit;
    HTuple objectModel3D, surfaceModel, objModelDiameter, diaOut;

    cout << "\nSet unit for points/vertices. 1 is m, 0,001 is mm: ";
    cin >> unit;

    HalconIO::readObjectModel3D(HTuple(argv[1]),unit,objectModel3D);

    // TODO: Calculate dia here and ask if it sounds correct, if not, then try again.
    objModelDiameter = HalconObjectModel::computeMaxDiameter(objectModel3D);

    cout << "Object diameter is" << objModelDiameter.ToString() << endl
            << "Make sure that the unit corresponds to what you expect from scene measurements" << endl << endl;

    if (!HalconObjectModel::hasPointNormals(objectModel3D) == 0){
        cout << "No point normals found. Computing." << endl;
        HalconObjectModel::computeSurfaceNormals(objectModel3D);
    }

    if (argv[3] == "custom") {

        cout << "---------------------------------------------------------"
             << "\n---------------------- CUSTOM PARAMS --------------------"
             << "\n---------------------------------------------------------" << endl << endl;

        float sampDist;
        cout << "Set relative sampling distance (default is 0.03): ";
        cin >> sampDist;

        float relSampDist, relStepSize, angleRes;
        cout << "Set relative sampling distance (default is 0.01): ";
        cin >> relSampDist;

        cout << "Set relative step size (default is 0.03): ";
        cin >> relStepSize;

        cout << "Set angle resolution (default is 30): ";
        cin >> angleRes;

        cout << endl;

        HTuple genParamName;
        genParamName.Append("model_invert_normals");
        genParamName.Append("pose_ref_rel_sampling_distance");
        genParamName.Append("feat_step_size_rel");
        genParamName.Append("feat_angle_resolution");

        HTuple genParamValue;
        genParamValue.Append("false");
        genParamValue.Append(relSampDist);
        genParamValue.Append(relStepSize);
        genParamValue.Append(angleRes);

        if (argv[4] == "edges") {
            genParamName.Append("train_3d_edges");
            genParamValue.Append("true");
        }

        HalconSurfaceModel::createSurfaceModel(objectModel3D, sampDist, genParamName, genParamValue, surfaceModel);

        //TODO: Saving of configurations in yaml or text


        const HTuple relSamplingDist(0.03); //0.05 default, might be too low on some models, such as MM1
        HTuple *surface_model = new HTuple();
    }

    else {

        cout << "---------------------------------------------------------"
             << "\n--------------------- DEFAULT PARAMS --------------------"
             << "\n---------------------------------------------------------" << endl << endl;

        float sampDist = 0.03;
        HTuple genParamName;
        genParamName.Append("model_invert_normals");
        genParamName.Append("pose_ref_rel_sampling_distance");
        genParamName.Append("feat_step_size_rel");
        genParamName.Append("feat_angle_resolution");

        HTuple genParamValue;
        genParamValue.Append("false");
        genParamValue.Append(0.01);
        genParamValue.Append(0.03);
        genParamValue.Append(30);

        if (argv[4] == "edges") {
            genParamName.Append("train_3d_edges");
            genParamValue.Append("true");
        }

        HalconSurfaceModel::createSurfaceModel(objectModel3D, sampDist, genParamName, genParamValue, surfaceModel);
    }

    HalconIO::writeSurfaceModel(HTuple(argv[2]),surfaceModel);

    HTuple sampledModelPoints;
    HalconSurfaceModel::getSampledModel(surfaceModel,sampledModelPoints);

    cout << "Number of points sampled: " << HalconObjectModel::getNumberOfPoints(sampledModelPoints) << endl;
    cout << "Diameter: " << HalconSurfaceModel::getDiameterSurfaceModel(surfaceModel).ToString() << " meters" << endl;

    ClearAllSurfaceModels();
    ClearAllObjectModel3d();
    cout << "Ending surface model creator." << endl;

    const char *fileName = "/home/eirik/catkin_ws/src/pose_estimator_pkg/pose_estimator/models/temp";
    HalconIO::writeObjectModel3DPly(fileName,sampledModelPoints);

    // TODO: CONVERT TO PCD INSTEAD, AND USE PCL TO VISUALIZE
    system("meshlab /home/eirik/catkin_ws/src/pose_estimator_pkg/pose_estimator/models/temp.ply");

    return 0;
}

// TODO: Save textfile with all surf model info
// TODO: Make a surf model inspector that checks keypoints, size compared to scene etc.