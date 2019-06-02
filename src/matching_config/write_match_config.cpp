//
// Created by eirik on 13.03.19.
//

#include <iostream>
#include <fstream>
#include <string.h>
#include <yaml-cpp/yaml.h>

// TODO: Include all matching settings here

// TODO: Make one function that can take an input value and append it to the text file
// In main, call this for each value to append. EASIER TO PORT TO GUI OR HTML THIS WAY
// Make option to save matching_config file
//
using namespace std;

void yamlEmitter(const char *filename, vector<string> settings, vector<string> values){

    ofstream ofile(filename);

    YAML::Emitter out;
    out << YAML::BeginMap;
    for (int i = 0; i < settings.size(); i++){
        out << YAML::Key << settings[i];
        out << YAML::Value << values[i];
    }
    out << YAML::EndMap;

    ofile << out.c_str();
}


int main(int argc, char** argv) {

    if (argc < 2 || argv[1] == "-h"){
        cout << "Syntax is: " << argv[0] << "/filename.yml" << endl;
    }

    vector<string> settings{"num_matches",
                            "max_overlap_dist_rel",
                            "scene_normal_computation",
                            "sparse_pose_refinement",
                            "score_type",
                            "pose_ref_use_scene_normals",
                            "dense_pose_refinement",
                            "pose_ref_num_steps",
                            "pose_ref_sub_sampling",
                            "pose_ref_dist_threshold_rel",
                            "pose_ref_scoring_dist_rel"};

    vector<string> values;
    for (int i = 0; i < settings.size(); i++) {
        string tmp_value;
        cout << settings[i] << ": ";
        cin >> tmp_value;
        values.emplace_back(tmp_value);
    }

    yamlEmitter(argv[1], settings, values);


}