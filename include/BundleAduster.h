//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_BUNDLEADUSTER_H
#define GDR_BUNDLEADUSTER_H

#include <vector>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <unordered_map>
#include <ceres/ceres.h>

#include "KeyPointInfo.h"
#include "cameraRGBD.h"
#include "Point3d.h"
#include "poseInfo.h"

namespace gdr {
    struct BundleAdjuster {
        template<class T>
        static Eigen::Matrix<T, 3, 4> getCameraIntr(T fx, T cx, T fy, T cy) {
            Eigen::Matrix<T, 3, 4> cameraIntr;
            cameraIntr.setZero();

            cameraIntr.col(0)[0] = fx;
            cameraIntr.col(1)[1] = fy;
            cameraIntr.col(2)[0] = cx;
            cameraIntr.col(2)[1] = cy;
            cameraIntr.col(2)[2] = T(1.0);
            return cameraIntr;
        }

        static const int dimPoint = 3;
        static const int dimPose = 3;
        static const int dimOrientation = 4;
        static const int quatStartIndex = 0;
        static const int cameraIntrStartIndex = 0;
        static const int cameraDim = 4;
        // size of each vector is 3
        // contains point coordinates being optimized
        std::vector<std::vector<double>> pointsXYZbyIndex;

        // size of each vector is 11:
        // tx, ty, tz,______ fx, cx, fy, cy
        // 0   1   2______   3   4   5   6
        // contains pose parameters being optimized
        std::vector<std::vector<double>> poseXYZqxyzwByPoseNumber;
        std::vector<std::vector<double>> poseFxCxFyCyByPoseNumber;
        std::vector<std::vector<double>> orientationsqxyzwByPoseNumber;

        // size should be equal to number of poses
        // contains intrinsics camera parameters
        std::vector<CameraRGBD> cameraModelByPoseNumber;

        std::vector<std::unordered_map<int, KeyPointInfo>> keyPointInfoByPoseNumberAndPointNumber;

        BundleAdjuster(const std::vector<Point3d> &points,
                       const std::vector<std::pair<Sophus::SE3d, CameraRGBD>> &absolutePoses,
                       const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointinfo);

        std::vector<Sophus::SE3d> optimizePointsAndPoses(int indexFixed = 0);

        std::vector<Point3d> getPointsGlobalCoordinatesOptimized() const;

        struct ReprojectionError {

            template<typename T>
            bool operator()(const T *const point,
                            const T *const pose,
                            const T *const orientation,
                            const T *const intrinsics,
                            T *residuals) const {

//                std::vector<T> qRaw = {pose[quatStartIndex], pose[quatStartIndex + 1], pose[quatStartIndex + 2],
//                                       pose[quatStartIndex + 3]};


                std::vector<T> qRaw = {orientation[0], orientation[1], orientation[2], orientation[3]};

                const T& fx = intrinsics[cameraIntrStartIndex];
                const T& cx = intrinsics[cameraIntrStartIndex + 1];
                const T& fy = intrinsics[cameraIntrStartIndex + 2];
                const T& cy = intrinsics[cameraIntrStartIndex + 3];

                Eigen::Quaternion<T> qQuat(qRaw.data());
                Sophus::Vector<T, 3> poseT(pose);

                Eigen::Matrix<T, 3, 4> cameraIntr = getCameraIntr<T>(fx, cx, fy, cy);

                Sophus::SE3<T> poseSE3;
                poseSE3.setQuaternion(qQuat);
                poseSE3.translation() = poseT;

                std::vector<T> pointRaw = {point[0], point[1], point[2], T(1.0)};
                Sophus::Vector<T, 4> point4d(pointRaw.data());

                Sophus::Vector<T, 4> pointCameraCoordinates = poseSE3.matrix() * point4d;
                Sophus::Vector<T, 3> imageCoordinates = cameraIntr * pointCameraCoordinates;

                T computedX = imageCoordinates[0] / imageCoordinates[2];
                T computedY = imageCoordinates[1] / imageCoordinates[2];

                residuals[0] = T(observedX) - computedX;
                residuals[1] = T(observedY) - computedY;

                // here is depth diff (for is not used)
                residuals[2] = T(0);

                return true;
            }

            double observedX;
            double observedY;
            double observedDepth;

            static ceres::CostFunction* Create(double newObservedX, double newObservedY, double newObservedDepth) {
                return (new ceres::AutoDiffCostFunction<ReprojectionError, 3, dimPoint, dimPose, dimOrientation, cameraDim>(
                        new ReprojectionError(newObservedX, newObservedY, newObservedDepth)));
            }

            ReprojectionError(double newObservedX, double newObservedY, double newObservedDepth)
                    : observedX(newObservedX), observedY(newObservedY), observedDepth(newObservedDepth) {}


        };


    };
}

#endif
