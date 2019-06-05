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

void setCustomParams(HTuple &genParamName, HTuple &genParamValue, float sampDist, string method){
    cout << "Set sampling distance (default is 0.03): ";
    cin >> sampDist;

    float relSampDist, relStepSize, angleRes;
    cout << "Set relative sampling distance (default is 0.01): ";
    cin >> relSampDist;

    cout << "Set relative step size (default is 0.03): ";
    cin >> relStepSize;

    cout << "Set angle resolution (default is 30): ";
    cin >> angleRes;

    cout << endl;

    genParamName.Append("model_invert_normals");
    genParamName.Append("pose_ref_rel_sampling_distance");
    genParamName.Append("feat_step_size_rel");
    genParamName.Append("feat_angle_resolution");

    genParamValue.Append("false");
    genParamValue.Append(relSampDist);
    genParamValue.Append(relStepSize);
    genParamValue.Append(angleRes);

    if (method == "edges") {
        genParamName.Append("train_3d_edges");
        genParamValue.Append("true");
    }
}


int main(int argc, char **argv){

    if (argc < 4 || argv[1] == "--h" || argv[1] == "-h"){
        printHelp(argc,argv);
        return -1;
    }

    float unit, sampDist;
    HTuple objectModel3D, surfaceModel, objModelDiameter, diaOut;
    HTuple genParamName, genParamValue;

    cout << "\nSet unit for points/vertices. HALCON converts to meters." <<
    endl << "The output diameter of the model should match what you " << endl
            << "expect to see in the measurements from the scene." << endl
            << "1 interprets numbers as meters, 0.001 as mm. Input unit here: " << endl;

    cin >> unit;

    HalconIO::readObjectModel3D(HTuple(argv[1]),unit,objectModel3D);

    objModelDiameter = HalconObjectModel::getDiameter(objectModel3D);
    cout << "Object diameter is " << objModelDiameter.ToString() << endl
            << "Make sure that the unit corresponds to what you expect from scene measurements" << endl << endl;

    if (!HalconObjectModel::hasPointNormals(objectModel3D)){
        cout << "No point normals found. Computing." << endl << endl;
        HalconObjectModel::computeSurfaceNormals(objectModel3D);
    }
    else {
        cout << "Surface normals found. Using existing." << endl << endl;
    }

    if (argv[3] == "custom") {

        cout << "---------------------------------------------------------"
             << "\n---------------------- CUSTOM PARAMS --------------------"
             << "\n---------------------------------------------------------" << endl << endl;

        setCustomParams(genParamName, genParamValue, sampDist, argv[4]);
        cout << "Starting model creation, this may take some time." << endl;
        HalconSurfaceModel::createSurfaceModel(objectModel3D, sampDist, genParamName, genParamValue, surfaceModel);

        //const HTuple relSamplingDist(0.03); //0.05 default, might be too low on some models, such as MM1
        //HTuple *surface_model = new HTuple();
    }

    else {

        cout << "---------------------------------------------------------"
             << "\n--------------------- DEFAULT PARAMS --------------------"
             << "\n---------------------------------------------------------" << endl << endl;

        HTuple sampDist = 0.03;
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

        if (argv[4] == std::string("edges")) {
            genParamName.Append("train_3d_edges");
            genParamValue.Append("true");
        }

        cout << "Starting model creation, this may take some time." << endl;
        HalconSurfaceModel::createSurfaceModel(objectModel3D, sampDist, genParamName, genParamValue, surfaceModel);
    }

    cout << "Done. Writing surface model" << endl;

    HalconIO::writeSurfaceModel(HTuple(argv[2]),surfaceModel);

    HTuple sampledModelPoints;
    HalconSurfaceModel::getSampledModel(surfaceModel,sampledModelPoints);
    cout << "Number of points sampled: " << HalconObjectModel::getNumberOfPoints(sampledModelPoints) << endl;
    cout << "Diameter: " << HalconSurfaceModel::getDiameterSurfaceModel(surfaceModel).ToString() << " mm" << endl;

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