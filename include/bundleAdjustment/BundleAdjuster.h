//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_BUNDLEADJUSTER_H
#define GDR_BUNDLEADJUSTER_H

#include <vector>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <unordered_map>
#include <ceres/ceres.h>

#include "IBundleAdjuster.h"
#include "keyPoints/KeyPointInfo.h"
#include "cameraModel/CameraRGBD.h"
#include "parametrization/SE3.h"
#include "parametrization/Point3d.h"
#include "readerDataset/readerTUM/PoseFullInfo.h"
#include "optimization/lossFunctions/LossFunctions.h"
#include "statistics/RobustEstimators.h"

namespace gdr {

    class BundleAdjuster : public IBundleAdjuster {

        bool printProgressToCout = true;

        void setPosesAndPoints(const std::vector<Point3d> &points,
                               const std::vector<std::pair<SE3, CameraRGBD>> &absolutePoses,
                               const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfo);

    public:

        bool getPrintProgressToCout() const;

        void setPrintProgressToCout(bool printProgress);

        std::vector<SE3> optimizePointsAndPoses(const std::vector<Point3d> &points,
                                                const std::vector<std::pair<SE3, CameraRGBD>> &absolutePoses,
                                                const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfo,
                                                int indexFixed = 0) override;

        std::vector<Point3d> getOptimizedPoints() const override;

    private:
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

        // TODO: learn what these parameters are used for in MAD estimators
        int n = -1;
        int p = -1;

        constexpr static const double factor = 1.4826;

        int maxNumberTreadsCeres = 6;
        int iterations = 50;

        std::function<double(double)> computeInitialScaleByMedian =
                [this](double medianError) {

                    double multiplier = 1.0;
                    if (n >= 0 && p >= 0) {
                        multiplier = 1 + 5.0 / (n - p);
                    }

                    return factor * multiplier * std::abs(medianError);

                };

        // size of each vector is 3
        // x, y, z
        // 0, 1, 2
        // contains point coordinates being optimized
        std::vector<std::vector<double>> pointsXYZbyIndex;

        // size of each vector is 3:
        // tx, ty, tz
        // 0   1   2
        // contains pose parameters being optimized
        std::vector<std::vector<double>> poseTxTyTzByPoseNumber;

        // size of each vector is 5:
        // fx, cx, fy, cy, keyPointScale
        // 0   1   2   3   4
        std::vector<std::vector<double>> poseFxCxFyCyScaleByPoseNumber;

        // size of each vector is 4:
        // qx, qy, qz, qw
        // 0   1   2    3
        // contains camera orientation being optimized
        std::vector<std::vector<double>> orientationsqxyzwByPoseNumber;

        // size should be equal to number of poses
        // contains intrinsics camera parameters
        std::vector<CameraRGBD> cameraModelByPoseNumber;

        // get median errors: reprojection OX and OY and Depth: [OX, OY, Depth]
        std::vector<double> getMedianErrorsXYDepth();

        // get vector of 3D reprojection errors (L2 distance between computed and "observed" points)
        std::vector<double> getL2Errors();

        /**
         * @param performNormalizing is true if errors should be divided by their deviation estimation
         * @returns two vector of errors:
         *      first is N-element vector of normalized (divided by expected standard deviation) reprojection errors in pixels
         *      second is N-element vector of normalized depth errors
         */
        std::pair<std::vector<double>, std::vector<double>>
        getNormalizedErrorsReprojectionAndDepth(bool performNormalizing = true);

        /** filter all errors and leave only those which satisfy |r_i/s_0| <= thresholdInlier
         *
         * @param r_n contains residues
         * @param s_0 residue divider parameter
         * @param thresholdInlier is used for inlier residue detection
         * @returns vector of inlier errors
         */
        std::vector<double> getInlierErrors(const std::vector<double> &r_n,
                                            double s_0,
                                            double thresholdInlier = 2.5);

        /** Compute inlier reprojection errors
         *
         * @param thresholdInlier is used for inlier residue detection
         * @returns two vector of errors, where each error r_i satisfies |r_i/s_0| <= thresholdInlier:
         *      where s_0 = factor * (1 + 5 / (n - p)) sqrt(med_i(r_i ^ 2 (theta)))
         *      first is N1-element vector of normalized (divided by expected standard deviation) reprojection errors in pixels
         *      second is N2-element vector of normalized depth errors
         */
        std::pair<std::vector<double>, std::vector<double>>
        getInlierNormalizedErrorsReprojectionAndDepth(double thresholdInlier = 2.5);


        /** Get deviation median based estimation for normalized reprojection (in pixels) and depth errors (in meters)
         *
         * @param threshold is used for inlier residue detection
         * @returns deviation estimations of reprojection and depth errors
         */
        std::pair<double, double> getSigmaReprojectionAndDepth(double threshold = 2.5);


        Sophus::SE3d getSE3TransformationMatrixByPoseNumber(int poseNumber) const;

        Eigen::Vector3d getPointVector3dByPointGlobalIndex(int pointGlobalIndex) const;

        Eigen::Vector4d getPointVector4dByPointGlobalIndex(int pointGlobalIndex) const;

        std::vector<std::unordered_map<int, KeyPointInfo>> keyPointInfoByPoseNumberAndPointNumber;


        struct ReprojectionWithDepthError {

            template<typename T>
            bool operator()(const T *const point,
                            const T *const pose,
                            const T *const orientation,
                            T *residuals) const {

                T fx = T(camera.getFx());
                T cx = T(camera.getCx());
                T fy = T(camera.getFy());
                T cy = T(camera.getCy());

                Eigen::Map<const Eigen::Quaternion<T>> orientationQuat(orientation);
                Eigen::Map<const Sophus::Vector<T, 3>> poseT(pose);

                Eigen::Matrix<T, 3, 4> cameraIntr = getCameraIntr<T>(fx, cx, fy, cy);

                Sophus::SE3<T> poseSE3;
                poseSE3.setQuaternion(orientationQuat);
                poseSE3.translation() = poseT;

                std::vector<T> pointRaw = {point[0], point[1], point[2], T(1.0)};
                Sophus::Vector<T, 4> point4d(pointRaw.data());

                Sophus::Vector<T, 4> pointCameraCoordinates = poseSE3.inverse().matrix() * point4d;
                Sophus::Vector<T, 3> imageCoordinates = cameraIntr * pointCameraCoordinates;

                T computedX = imageCoordinates[0] / imageCoordinates[2];
                T computedY = imageCoordinates[1] / imageCoordinates[2];
                T computedDepth = pointCameraCoordinates[2];

                T resX = T(observedX) - computedX;
                T resY = T(observedY) - computedY;

                // reprojection error computation (noise modeled as sigma = k * scale)
                {
                    T normResidual = ceres::sqrt(resX * resX + resY * resY);
                    normResidual = LossFunctionHuber::evaluate<T>(normResidual / T(deviationDividerReproj),
                                                                  T(deviationEstimationNormalizedReproj));
                    residuals[0] = ceres::sqrt(normResidual);
                }

                // depth error computation (noise modeled as sigma = a * depth^2 [meters])
                {
                    T normResidual = ceres::abs(T(observedDepth) - computedDepth);
//                    normResidual = LossFunctionHuber::evaluate<T>(normResidual / T(deviationDividerDepth),
//                                                                  T(deviationEstimationNormalizedDepth));
                    normResidual = LossFunctionTukey::evaluate<T>(normResidual / T(deviationDividerDepth),
                                                                  T(deviationEstimationNormalizedDepth));
                    residuals[1] = ceres::sqrt(normResidual);
                }

                return true;
            }

            double scaleKeyPoint;
            double observedX;
            double observedY;
            double observedDepth;

            double deviationDividerReproj;
            double deviationDividerDepth;

            double deviationEstimationNormalizedReproj;
            double deviationEstimationNormalizedDepth;

            double medianResidualReproj;
            double medianResidualDepth;

            CameraRGBD camera;

            static ceres::CostFunction *Create(double newObservedX, double newObservedY, double newObservedDepth,
                                               double scale,
                                               const CameraRGBD &camera,
                                               double estNormalizedReproj, double estNormalizedDepth,
                                               double devDividerReproj, double devDividerDepth,
                                               double medianResReproj, double medianResDepth);

            ReprojectionWithDepthError(double newObservedX, double newObservedY, double newObservedDepth,
                                       double scale,
                                       const CameraRGBD &cameraRgbd,
                                       double devNormalizedEstReproj, double devNormalizedEstDepth,
                                       double devDividerReproj, double devDividerDepth,
                                       double medianResReproj, double medianResDepth);


        };

    public:
        int getMaxNumberIterations() const;
        int getMaxNumberThreads() const;

        void setMaxNumberIterations(int iterationsNumber);
        void setMaxNumberThreads(int numberOfThreads);

    };
}

#endif
