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
#include "parametrization/cameraRGBD.h"
#include "parametrization/SE3.h"
#include "parametrization/Point3d.h"
#include "readerTUM/PoseFullInfo.h"
#include "optimization/lossFunctions/LossHuber.h"
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

//        BundleAdjuster(const std::vector<Point3d> &points,
//                       const std::vector<std::pair<SE3, CameraRGBD>> &absolutePoses,
//                       const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointinfo);

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
//        static const int quatStartIndex = 0;
//        static const int cameraIntrStartIndex = 0;
//        static const int cameraDim = 4;
//        static const int scaleStartIndex = 4;
        int n = -1;
        int p = -1;
        constexpr static const double factor = 1.4826;
        constexpr static const double parameterScaleMultiplier = 1.0;

        std::function<double(double, double)> dividerReprojectionError =
                [](double scale, double linearParameterNoiseModelReprojection) {
                    return linearParameterNoiseModelReprojection * scale;
                };
        std::function<double(double, double)> dividerDepthError =
                [](double depth, double quadraticParameterNoiseModelDepth) {
                    double squaredDepth = std::pow(depth, 2.0);
                    return quadraticParameterNoiseModelDepth * squaredDepth;
                };

        std::function<double(double, double, double)> reprojectionErrorNormalizer =
                [this](double reprojectionError, double keyPointScale, double linearParameterNoiseModelReprojection) {
                    return reprojectionError /
                           dividerReprojectionError(keyPointScale, linearParameterNoiseModelReprojection);
                };

        std::function<double(double, double, double)> depthErrorNormalizer =
                [this](double depthError, double keyPointDepth, double quadraticParameterNoiseModelDepth) {
                    return depthError / dividerDepthError(keyPointDepth, quadraticParameterNoiseModelDepth);
                };

        std::function<double(double)> computeInitialScaleByMedian =
                [this](double medianError) {

                    double multiplier = 1.0;
                    if (n >= 0 && p >= 0) {
                        multiplier = 1 + 5.0 / (n - p);
                    }

                    return factor * multiplier * std::abs(medianError);

                };

        double defaultQuadraticParameterNoiseModelDepth = 3.331e-3;
        double defaultLinearParameterNoiseModelReprojectionBA = 0.987;

        // size of each vector is 3
        // contains point coordinates being optimized
        std::vector<std::vector<double>> pointsXYZbyIndex;

        // size of each vector is 11:
        // tx, ty, tz
        // 0   1   2
        // contains pose parameters being optimized
        std::vector<std::vector<double>> poseTxTyTzByPoseNumber;
        // size of each vector is 5:
        // fx, cx, fy, cy, keyPointScale
        // 0   1   2   3   4
        // contains pose parameters being optimized
        std::vector<std::vector<double>> poseFxCxFyCyScaleByPoseNumber;
        std::vector<std::vector<double>> orientationsqxyzwByPoseNumber;

        // size should be equal to number of poses
        // contains intrinsics camera parameters
        std::vector<CameraRGBD> cameraModelByPoseNumber;


        // get median errors: reprojection OX and OY and Depth: [OX, OY, Depth]
        std::vector<double> getMedianErrorsXYDepth();


        // get vector of 3D reprojection errors (L2 distance between computed and "observed" points)
        std::vector<double> getL2Errors();

        // get two vector of errors:
        // first is N-element vector of normalized (divided by expected standard deviation) reprojection errors in pixels
        // second is N-element vector of normalized depth errors
        std::pair<std::vector<double>, std::vector<double>>
        getNormalizedErrorsReprojectionAndDepth(bool performNormalizing = true);

        // filter all errors and leave only those which satisfy |r_i/s_0| <= thresholdInlier

        std::vector<double> getInlierErrors(const std::vector<double> &r_n,
                                            double s_0,
                                            double thresholdInlier = 2.5);

        // get two vector of errors, each error r_i should pass |r_i/s_0| <= thresholdInlier:
        //      where s_0 = factor * (1 + 5 / (n - p)) sqrt(med_i(r_i ^ 2 (theta)))
        // first is N1-element vector of normalized (divided by expected standard deviation) reprojection errors in pixels
        // second is N2-element vector of normalized depth errors
        std::pair<std::vector<double>, std::vector<double>>
        getInlierNormalizedErrorsReprojectionAndDepth(double thresholdInlier = 2.5);


        // get deviation median based estimation for normalized reprojection (in pixels) and depth errors (in meters)
        // @parameter threshold is used to determine which errors r_i are inliers by initial scale s_0 as
        // if they satisfy |r_i / s_0| < threshold
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
                T fx = T(camera.fx);
                T cx = T(camera.cx);
                T fy = T(camera.fy);
                T cy = T(camera.cy);

                Eigen::Map<const Eigen::Quaternion<T>> qQuat(orientation);
                Eigen::Map<const Sophus::Vector<T, 3>> poseT(pose);

                Eigen::Matrix<T, 3, 4> cameraIntr = getCameraIntr<T>(fx, cx, fy, cy);

                Sophus::SE3<T> poseSE3;
                poseSE3.setQuaternion(qQuat);
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

//                LossHuber<T> lossFunctionRobust;
                // reprojection error computation (noise modeled as sigma = k * scale)
                {
                    T normResidual = ceres::sqrt(resX * resX + resY * resY);
                    normResidual = ILossFunction::evaluate<T>(normResidual / T(deviationDividerReproj),
                                                              T(deviationEstimationNormalizedReproj));
                    residuals[0] = ceres::sqrt(normResidual);
                }

                // depth error computation (noise modeled as sigma = a * depth^2 [meters])
                {
                    T normResidual = ceres::abs(T(observedDepth) - computedDepth);
                    normResidual = ILossFunction::evaluate<T>(normResidual / T(deviationDividerDepth),
                                                              T(deviationEstimationNormalizedDepth));
                    residuals[1] = ceres::sqrt(normResidual);
                }

                return true;
            }

            int width = 640;
            int height = 480;
            double scaleKeyPoint;
            double radii;
            double observedX;
            double observedY;
            double observedDepth;

            double deviationDividerReproj;
            double deviationDividerDepth;

            double deviationEstimationNormalizedReproj;
            double deviationEstimationNormalizedDepth;

            double medianResidualReproj;
            double medianResidualDepth;

            double quadraticParameterNoiseModelDepth = 3.331e-3;
            double linearParameterNoiseModelReprojection = 0.987;

            CameraRGBD camera;

            static ceres::CostFunction *Create(double newObservedX, double newObservedY, double newObservedDepth,
                                               double scale,
                                               const CameraRGBD &camera,
                                               double estNormalizedReproj, double estNormalizedDepth,
                                               double devDividerReproj, double devDividerDepth,
                                               double medianResReproj, double medianResDepth) {
                return (new ceres::AutoDiffCostFunction<ReprojectionWithDepthError, 2, dimPoint, dimPose, dimOrientation>(
                        new ReprojectionWithDepthError(newObservedX, newObservedY, newObservedDepth,
                                                       scale, camera,
                                                       estNormalizedReproj, estNormalizedDepth,
                                                       devDividerReproj, devDividerDepth,
                                                       medianResReproj, medianResDepth)));
            }

            ReprojectionWithDepthError(double newObservedX, double newObservedY, double newObservedDepth,
                                       double scale,
                                       const CameraRGBD &cameraRgbd,
                                       double devNormalizedEstReproj, double devNormalizedEstDepth,
                                       double devDividerReproj, double devDividerDepth,
                                       double medianResReproj, double medianResDepth)
                    : observedX(newObservedX),
                      observedY(newObservedY),
                      observedDepth(newObservedDepth),
                      scaleKeyPoint(scale),
                      camera(cameraRgbd),
                      deviationEstimationNormalizedReproj(devNormalizedEstReproj),
                      deviationEstimationNormalizedDepth(devNormalizedEstDepth),
                      deviationDividerReproj(devDividerReproj),
                      deviationDividerDepth(devDividerDepth),
                      medianResidualReproj(medianResReproj),
                      medianResidualDepth(medianResDepth) {}


        };

    };
}

#endif
