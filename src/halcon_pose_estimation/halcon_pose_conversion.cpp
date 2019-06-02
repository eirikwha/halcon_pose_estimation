//
// Created by eirik on 04.03.19.
//

#include <kuka_eki_msgs/KrlPos.h>
#include "halcon_pose_estimation/halcon_pose_conversion.h"

namespace HalconPoseConversion {

    void poseToHomMat(HTuple &pose, HTuple &homMat3D) {
        try {
            PoseToHomMat3d(pose, &homMat3D);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void poseRotToQuat(HTuple &pose, HTuple &quat) {
        try {
            PoseToQuat(pose, &quat);
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }

    void poseTrans(HTuple &pose, HTuple &trans) {
        for (int i = 0; i <= 2; i++) {
            try {
                TupleInsert(trans, i, pose[i], &trans);
            }
            catch (HException &exc) {
                cout << exc.ErrorMessage() << endl;
            }
        }
    }

    void poseToEigen4fPose(HTuple pose, Eigen::Matrix4f &t) {

        HTuple homMat3D;
        PoseToHomMat3d(pose, &homMat3D);

        t(0, 0) = (double) homMat3D[0];
        t(0, 1) = (double) homMat3D[1];
        t(0, 2) = (double) homMat3D[2];
        t(0, 3) = (double) homMat3D[3];
        t(1, 0) = (double) homMat3D[4];
        t(1, 1) = (double) homMat3D[5];
        t(1, 2) = (double) homMat3D[6];
        t(1, 3) = (double) homMat3D[7];
        t(2, 0) = (double) homMat3D[8];
        t(2, 1) = (double) homMat3D[9];
        t(2, 2) = (double) homMat3D[10];
        t(2, 3) = (double) homMat3D[11];
        t(3, 0) = 0;
        t(3, 1) = 0;
        t(3, 2) = 0;
        t(3, 3) = 1;
    }

    void poseToGeometryMsgTuple(HTuple &pose, HTuple &geometryMsg) {
        HTuple rotQuat;
        poseRotToQuat(pose, rotQuat);

        for (int i = 0; i <= 2; i++) {
            try {
                TupleInsert(geometryMsg, i, pose[i], &geometryMsg);
            }
            catch (HException &exc) {
                cout << exc.ErrorMessage() << endl;
            }
        }
        for (int i = 3; i <= 6; i++) {
            try {
                TupleInsert(geometryMsg, i, rotQuat[i - 3], &geometryMsg);
            }
            catch (HException &exc) {
                cout << exc.ErrorMessage() << endl;
            }
        }
    }

    void poseToGeometryMsgPoseStamped(HTuple &pose, geometry_msgs::PoseStamped &poseOut, std::string &frameID) {

        HTuple quat;
        poseRotToQuat(pose, quat);

        poseOut.pose.position.x = pose[0];
        poseOut.pose.position.y = pose[1];
        poseOut.pose.position.z = pose[2];
        poseOut.pose.orientation.x = quat[1];
        poseOut.pose.orientation.y = quat[2];
        poseOut.pose.orientation.z = quat[3];
        poseOut.pose.orientation.w = quat[0];

        poseOut.header.frame_id = frameID;
        poseOut.header.stamp = ros::Time::now();
    }

    void poseToGeometryMsgPose(HTuple &pose, geometry_msgs::Pose &poseOut) {

        HTuple quat;
        poseRotToQuat(pose, quat);

        poseOut.position.x = pose[0];
        poseOut.position.y = pose[1];
        poseOut.position.z = pose[2];
        poseOut.orientation.x = quat[1];
        poseOut.orientation.y = quat[2];
        poseOut.orientation.z = quat[3];
        poseOut.orientation.w = quat[0];

    }

    void poseToKukaXYZRPY(HTuple &pose, kuka_eki_msgs::KrlPos &poseOut) {
        try {
            HTuple tmpPose;
            ConvertPoseType(pose, "Rp+T", "abg", "point", &tmpPose);

            poseOut.x = tmpPose[0];
            poseOut.y = tmpPose[1];
            poseOut.z = tmpPose[2];
            poseOut.a = tmpPose[3];
            poseOut.b = tmpPose[4];
            poseOut.c = tmpPose[5];
            poseOut.s = 0;
            poseOut.t = 0;
        }
        catch (HException &exc) {
            cout << exc.ErrorMessage() << endl;
        }
    }
}