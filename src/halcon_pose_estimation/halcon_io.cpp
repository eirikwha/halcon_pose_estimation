//
// Created by eirik on 04.03.19.
//

#include "halcon_pose_estimation/halcon_io.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "matching_config/read_match_config.h"
#include <string>
#include <yaml-cpp/yaml.h>

namespace HalconIO {

    HTuple readObjectModel3D(HTuple loadPath, float unit, HTuple &objectModel3D) {
        cout << "Looking for object model in " << loadPath.ToString() << endl;
        try {
            HTuple *status = new HTuple();
            ReadObjectModel3d(loadPath, unit, HTuple(), HTuple(), &objectModel3D, status);
            cout << "Object model successfully loaded.\n" << endl;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
        return objectModel3D;
    }

    HTuple readObjectModel3DWidthMap(HTuple loadPath, float unit, int cloudWidth, HTuple &objectModel3D) {
        cout << "Looking for object model in " << loadPath.ToString() << endl;
        try {
            HTuple *status = new HTuple();
            ReadObjectModel3d(loadPath, unit, HTuple("xyz_map_width"), HTuple(cloudWidth), &objectModel3D, status);
            cout << "Object model successfully loaded.\n" << endl;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    HTuple readSurfaceModel(HTuple loadPath, HTuple &surfaceModelID) {
        cout << "Looking for surface model in " << loadPath.ToString() << endl;
        try {
            ReadSurfaceModel(loadPath, &surfaceModelID);
            cout << "Surface model sucessfully loaded.\n" << endl;
        }
        catch (HException &except) {
            cout << loadPath.ToString() << "\n" << except.ErrorMessage().Text() << endl;
            cout << "Run createHalconSurfaceModel() to create model or specify another filepath\n" << endl;
        }
        return surfaceModelID;
    }

    void writeObjectModel3DPly(HTuple outputFileName, HTuple &objectModel3D) {
        try {
            WriteObjectModel3d(objectModel3D, "ply", outputFileName, "invert_normals", 0);
            cout << "Object model saved in " << outputFileName.ToString() << endl << endl;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void writeSurfaceModel(HTuple outputFileName, HTuple &surfaceModel3D) {
        try {
            WriteSurfaceModel(surfaceModel3D, outputFileName);
            cout << "Surface model saved in " << outputFileName.ToString() << endl << endl;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void createModelFromPoints(const HTuple &x, const HTuple &y, const HTuple &z, HTuple &objectModel3D) {
        try {
            GenObjectModel3dFromPoints(x, y, z, &objectModel3D);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void createModelFromPCLPointXYZ(HTuple &x, HTuple &y, HTuple &z, HTuple &objectModel3D,
                                    pcl::PointCloud<pcl::PointXYZ> cloud) {

        // TODO: TOO SLOW COMPARED TO HALCONBRIDGE, MAYBE TRY OPENMP??
        #pragma omp parallel for
        for (int i = 0; i < cloud.points.size(); i += 100) // i+=100, subsample to avoid crash.
        {
            x.Append(cloud.points[i].x);
            y.Append(cloud.points[i].y);
            z.Append(cloud.points[i].z);
        }

        createModelFromPoints(x, y, z, objectModel3D);
    }

    void readGenParams(const char *filename, HTuple &genParamName, HTuple &genParamValue) {
        int numLines = countLines(filename);
        for (int i = 0; i < numLines; i++) {

            string name = getName(filename, i);
            char cstr[name.size() + 1];
            strcpy(cstr, name.c_str());

            genParamName.Append(HTuple(cstr));

            string value = getValue(filename, i);
            char fstr[value.size() + 1];
            strcpy(fstr, value.c_str());

            genParamValue.Append(HTuple(fstr));
        }
    }

    void readGenParamValues(const char *filename, HTuple &genParamValue) {
        int numLines = countLines(filename);
        for (int i = 0; i < numLines; i++) {

            string value = readLine(filename, i);
            char fstr[value.size() + 1];
            strcpy(fstr, value.c_str());

            genParamValue.Append(HTuple(fstr));
        }
    }


    void readYamlConfig(const char *filePath, HTuple &genParamName, HTuple &genParamValue) {
        YAML::Node config = YAML::LoadFile(filePath);
        vector<string> names, values;
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
            names.emplace_back(it->first.as<string>());
            values.emplace_back(it->second.as<string>());
        }

        for (int i = 0; i < names.size(); i++) {
            char fstr[names[i].size() + 1];
            strcpy(fstr, names[i].c_str());
            genParamName.Append(HTuple(fstr));
            cout << genParamName.ToString() << endl;
        }

        for (int i = 0; i < values.size(); i++) {
            char fstr[values[i].size() + 1];
            strcpy(fstr, values[i].c_str());
            genParamValue.Append(HTuple(fstr));
            cout << genParamValue.ToString() << endl;
        }
    }
}