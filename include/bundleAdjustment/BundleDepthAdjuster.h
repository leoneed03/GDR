//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_BUNDLEDEPTHADJUSTER_H
#define GDR_BUNDLEDEPTHADJUSTER_H

#include <vector>
#include <unordered_map>
#include <thread>

#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include "ceres/ceres.h"

#include "BundleAdjuster.h"
#include "keyPoints/KeyPointInfo.h"
#include "cameraModel/CameraRGBD.h"
#include "parametrization/SE3.h"
#include "parametrization/Point3d.h"
#include "parametrization/PoseFullInfo.h"
#include "statistics/RobustEstimators.h"

namespace gdr {

    class BundleDepthAdjuster : public BundleAdjuster {

        static constexpr double thresholdInlierDefault = 2.5;

        bool printProgressToCout = false;

        void setPosesAndPoints(const std::vector<Point3d> &points,
                               const std::vector<std::pair<SE3, CameraRGBD>> &posesCameraToWorld,
                               const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfo);

    public:

        bool getPrintProgressToCout() const;

        void setPrintProgressToCout(bool printProgress);

        std::vector<SE3> optimizePointsAndPoses(const std::vector<Point3d> &points,
                                                const std::vector<std::pair<SE3, CameraRGBD>> &posesCameraToWorld,
                                                const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfo,
                                                int indexFixed,
                                                bool &success) override;

        std::vector<Point3d> getOptimizedPoints() const override;

    private:
        static const int dimPoint = 3;
        static const int dimPose = 3;
        static const int dimOrientation = 4;

        int n = -1;
        int p = -1;

        static constexpr double factor = 1.4826;

        int maxNumberTreadsCeres = static_cast<int>(std::thread::hardware_concurrency());
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
        std::vector<std::vector<double>> poseWorldToCameraTxTyTzByPoseNumber;

        // size of each vector is 4:
        // qx, qy, qz, qw
        // 0   1   2    3
        // contains camera orientation being optimized
        std::vector<std::vector<double>> orientationWorldToCameraqxyzwByPoseNumber;

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
                                            double thresholdInlier = thresholdInlierDefault);

        /** Compute inlier reprojection errors
         *
         * @param thresholdInlier is used for inlier residue detection
         * @returns two vector of errors, where each error r_i satisfies |r_i/s_0| <= thresholdInlier:
         *      where s_0 = factor * (1 + 5 / (n - p)) sqrt(med_i(r_i ^ 2 (theta)))
         *      first is N1-element vector of normalized (divided by expected standard deviation) reprojection errors in pixels
         *      second is N2-element vector of normalized depth errors
         */
        std::pair<std::vector<double>, std::vector<double>>
        getInlierNormalizedErrorsReprojectionAndDepth(double thresholdInlier = thresholdInlierDefault);


        /** Get deviation median based estimation for normalized reprojection (in pixels) and depth errors (in meters)
         *
         * @param threshold is used for inlier residue detection
         * @returns deviation estimations of reprojection and depth errors
         */
        std::pair<double, double> getSigmaReprojectionAndDepth(double threshold = thresholdInlierDefault);


        Sophus::SE3d getSE3TransformationMatrixByPoseNumber(int poseNumber) const;

        Eigen::Vector3d getPointVector3dByPointGlobalIndex(int pointGlobalIndex) const;

        Eigen::Vector4d getPointVector4dByPointGlobalIndex(int pointGlobalIndex) const;

        std::vector<std::unordered_map<int, KeyPointInfo>> keyPointInfoByPoseNumberAndPointNumber;


        struct DepthOnlyResidual {

            template<typename T>
            bool operator()(const T *const point,
                            const T *const pose,
                            const T *const orientation,
                            T *residuals) const {

                Sophus::Vector<T, 3> pointInLocalCoordinates =
                        getPointLocalCameraSystemCoordinates<T>(point,
                                                                pose,
                                                                orientation);

                T &computedDepth = pointInLocalCoordinates[2];

                // depth error computation (noise modeled as sigma = a * depth^2 [meters])
                {
                    T normResidual = T(observedDepth) - computedDepth;
                    normResidual = normResidual / T(deviationDividerDepth);
                    residuals[0] = normResidual;
                }

                return true;
            }

            double observedDepth;

            double deviationDividerDepth;

            double deviationEstimationNormalizedDepth;

            double medianResidualDepth;

            const CameraRGBD &camera;

            static ceres::CostFunction *Create(double newObservedDepth,
                                               const CameraRGBD &camera,
                                               double estNormalizedDepth,
                                               double devDividerDepth,
                                               double medianResDepth);

            DepthOnlyResidual(double newObservedDepth,
                              const CameraRGBD &cameraRgbd,
                              double devNormalizedEstDepth,
                              double devDividerDepth,
                              double medianResDepth);
        };

        template<class T>
        Sophus::Vector<T, 3> static getPointLocalCameraSystemCoordinates(
                const T *const point,
                const T *const poseWorldToCameraTranslation,
                const T *const poseWorldToCameraOrientation) {

            Eigen::Map<const Sophus::SO3<T>> orientationWtoC(poseWorldToCameraOrientation);
            Eigen::Map<const Sophus::Vector<T, 3>> translationWtoC(poseWorldToCameraTranslation);

            Eigen::Map<const Sophus::Vector<T, 3>> point3d(point);
            Sophus::Vector<T, 3> pointCameraCoordinates = orientationWtoC * point3d + translationWtoC;

            return pointCameraCoordinates;
        }

        struct ReprojectionOnlyResidual {

            template<typename T>
            bool operator()(const T *const point,
                            const T *const poseTranslationWorldToCamera,
                            const T *const poseOrientationWorldToCamera,
                            T *residuals) const {

                Sophus::Vector<T, 3> pointInLocalCameraCoordinates =
                        getPointLocalCameraSystemCoordinates<T>(point,
                                                                poseTranslationWorldToCamera,
                                                                poseOrientationWorldToCamera);

                Sophus::Vector<T, 2> pointInImageCoordinatesNormalized =
                        camera.projectUsingIntrinsics<T>(pointInLocalCameraCoordinates);


                Eigen::Map<const Sophus::SO3<T>> orientationWorldToCamera(poseOrientationWorldToCamera);
                Eigen::Map<const Sophus::Vector<T, 3>> translationWorldToCamera(poseTranslationWorldToCamera);
                Eigen::Map<const Sophus::Vector<T, 3>> point3d(point);

                T &computedX = pointInImageCoordinatesNormalized[0];
                T &computedY = pointInImageCoordinatesNormalized[1];

                T resX = T(observedX) - computedX;
                T resY = T(observedY) - computedY;

                residuals[0] = resX / T(deviationDividerReproj);
                residuals[1] = resY / T(deviationDividerReproj);

                return true;
            }

            double observedX;
            double observedY;
            double scaleKeyPoint;

            double deviationDividerReproj;

            double deviationEstimationNormalizedReproj;
            double medianResidualReproj;
            double medianResidualDepth;

            const CameraRGBD &camera;

            static ceres::CostFunction *Create(double newObservedX,
                                               double newObservedY,
                                               double scale,
                                               const CameraRGBD &camera,
                                               double estNormalizedReproj,
                                               double devDividerReproj,
                                               double medianResReproj);

            ReprojectionOnlyResidual(double newObservedX,
                                     double newObservedY,
                                     double scale,
                                     const CameraRGBD &cameraRgbd,
                                     double devNormalizedEstReproj,
                                     double devDividerReproj,
                                     double medianResReproj);

        };

    public:
        int getMaxNumberIterations() const;

        int getMaxNumberThreads() const;

        void setMaxNumberIterations(int iterationsNumber);

        void setMaxNumberThreads(int numberOfThreads);

    };
}

#endif
