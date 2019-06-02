//
// Created by eirik on 05.03.19.
//

#include "halcon_pose_estimation/pose_result_handler.h"

namespace PoseResultHandler {

// The HTuple calib1_pose is the result HTuple from FindSurfaceModel()

    HTuple resultIterator;

    int getNumMatches(HTuple &matchPoses) { // maybe better with &calib1_pose here CHECK!!
        try {
            int numMatches = (matchPoses.Length()) / 7;
            return numMatches;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void getSinglePose(HTuple &matchingResultID, int resultNumber, HTuple &singlePoseOut) {
        try {
            GetSurfaceMatchingResult(matchingResultID, "pose", resultNumber, &singlePoseOut);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void getKeyPoints(HTuple &matchingResultID, int resultNumber, HTuple keyPointsOut) {
        try {
            GetSurfaceMatchingResult(matchingResultID, "key_points", resultNumber, &keyPointsOut);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void getSampledEdges(HTuple &matchingResultID, int resultNumber, HTuple sampledEdgesOut) {
        try {
            GetSurfaceMatchingResult(matchingResultID, "sampled_edges", resultNumber, &sampledEdgesOut);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void getSampledScene(HTuple &matchingResultID, int resultNumber, HTuple sampledSceneOut) {
        try {
            GetSurfaceMatchingResult(matchingResultID, "sampled_scene", resultNumber, &sampledSceneOut);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void printPoses(HTuple &matchingResultID, int numMatches) {

        for (int i = 0; i <= numMatches - 1; i++) {
            try {
                GetSurfaceMatchingResult(matchingResultID, "pose", i, &resultIterator);

                cout << "Pose" << i + 1 << ": " << resultIterator.ToString() << endl;
            }
            catch (HException &exc) {
                cout << exc.ErrorMessage() << endl;
            }
        }

    }

    void printScoreUnrefined(HTuple &matchingResultID, int numMatches) {
        for (int i = 0; i <= numMatches - 1; i++) {
            try {
                GetSurfaceMatchingResult(matchingResultID, "score", i, &resultIterator);

                cout << "Matching score, unrefined: " << resultIterator.ToString() << endl;
            }
            catch (HException &exc) {
                cout << exc.ErrorMessage() << endl;
            }
        }
    }

    void printScoreRefined(HTuple &matchingResultID, int numMatches) {
        for (int i = 0; i <= numMatches - 1; i++) {
            try {
                GetSurfaceMatchingResult(matchingResultID, "score", i, &resultIterator);

                cout << "Matching score, refined: " << resultIterator.ToString() << endl;
            }
            catch (HException &exc) {
                cout << exc.ErrorMessage() << endl;
            }
        }
    }

    void printPosesAndScores(HTuple &matchingResultID, int numMatches) {
        for (int i = 0; i <= numMatches - 1; i++) {
            const HTuple resultIndex(i);
            try {

                cout << "----------------------------------------"
                     << endl << endl;

                GetSurfaceMatchingResult(matchingResultID, "pose", resultIndex, &resultIterator); //NOT INDIVIDUAL POSES
                cout << "Pose " << i + 1 << ": " << resultIterator.ToString() << endl;

                GetSurfaceMatchingResult(matchingResultID, "score_unrefined", resultIndex, &resultIterator);
                cout << "Score, before refinement: " << resultIterator.ToString() << endl;

                GetSurfaceMatchingResult(matchingResultID, "score_refined", resultIndex, &resultIterator);
                cout << "Score, after refinement: " << resultIterator.ToString() << endl << endl;

                cout << "----------------------------------------"
                     << endl;
            }
            catch (HException &exc) {
                cout << exc.ErrorMessage() << endl;

            }
        }
    }
}
