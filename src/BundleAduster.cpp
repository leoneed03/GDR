//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "BundleAduster.h"

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

namespace gdr {

    BundleAdjuster::BundleAdjuster(const std::vector<Point3d> &points,
                                   const std::vector<std::pair<Sophus::SE3d, CameraRGBD>> &absolutePoses,
                                   const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointinfo) {
        assert(keyPointinfo.size() == absolutePoses.size());
        assert(!absolutePoses.empty());
        assert(absolutePoses.size() > 0);
        double widthAssert = 640;
        double heightAssert = 480;
        std::cout << "poses: " << absolutePoses.size() << std::endl;
        for (const auto &point: points) {
            pointsXYZbyIndex.push_back(point.getVectorPointXYZ());
        }

        for (const auto &mapIntInfo: keyPointinfo) {
            for (const auto &pairIntInfo: mapIntInfo) {
                assert(pairIntInfo.second.getX() < widthAssert && pairIntInfo.second.getX() >= 0);
                assert(pairIntInfo.second.getY() < heightAssert && pairIntInfo.second.getY() >= 0);
            }
        }
        keyPointInfoByPoseNumberAndPointNumber = keyPointinfo;


        for (const auto &pose: absolutePoses) {
            const auto &translation = pose.first.translation();
            const auto &rotationQuat = pose.first.unit_quaternion();
            const auto &cameraIntr = pose.second;
            poseTxTyTzByPoseNumber.push_back({translation[0], translation[1], translation[2]});
            poseFxCxFyCyScaleByPoseNumber.push_back({cameraIntr.fx, cameraIntr.cx, cameraIntr.fy, cameraIntr.cy});
            orientationsqxyzwByPoseNumber.push_back(
                    {rotationQuat.x(), rotationQuat.y(), rotationQuat.z(), rotationQuat.w()});

            cameraModelByPoseNumber.push_back(pose.second);
            assert(poseTxTyTzByPoseNumber[poseTxTyTzByPoseNumber.size() - 1].size() == dimPose);
        }

        assert(pointsXYZbyIndex.size() == points.size());
        assert(keyPointinfo.size() == keyPointInfoByPoseNumberAndPointNumber.size());
        assert(absolutePoses.size() == poseTxTyTzByPoseNumber.size());

    }

    std::vector<Sophus::SE3d> BundleAdjuster::optimizePointsAndPoses(int indexFixed) {


        ceres::Problem problem;
        ceres::LocalParameterization *quaternionLocalParameterization =
                new ceres::EigenQuaternionParameterization;
        std::cout << "started BA" << std::endl;

        assert(orientationsqxyzwByPoseNumber.size() == poseTxTyTzByPoseNumber.size());
        assert(!orientationsqxyzwByPoseNumber.empty());
        for (int poseIndex = 0; poseIndex < poseTxTyTzByPoseNumber.size(); ++poseIndex) {

            std::cout << "init pose " << poseIndex << std::endl;
            auto &pose = poseTxTyTzByPoseNumber[poseIndex];
            auto &orientation = orientationsqxyzwByPoseNumber[poseIndex];

            assert(poseIndex < keyPointInfoByPoseNumberAndPointNumber.size());

            for (const auto &indexAndKeyPointInfo: keyPointInfoByPoseNumberAndPointNumber[poseIndex]) {
                int pointIndex = indexAndKeyPointInfo.first;
                assert(pointIndex >= 0 && pointIndex < pointsXYZbyIndex.size());

                auto &point = pointsXYZbyIndex[pointIndex];
                auto &intr = poseFxCxFyCyScaleByPoseNumber[poseIndex];
                assert(intr.size() == cameraDim);
//                auto& observedKeyPoints = keyPointInfoByPoseNumberAndPointNumber[poseIndex];
                const auto &keyPointInfo = indexAndKeyPointInfo.second;

                double widthAssert = 640;
                double heightAssert = 480;
                double observedX = keyPointInfo.getX();
                double observedY = keyPointInfo.getY();
//                double observedX = width - keyPointInfo.getX();
//                double observedY = height - keyPointInfo.getY();

                assert(observedX > 0 && observedX < widthAssert);
                assert(observedY > 0 && observedY < heightAssert);

                ceres::CostFunction *cost_function =
                        ReprojectionError::Create(observedX, observedY, keyPointInfo.getDepth());
                problem.AddResidualBlock(cost_function,
                                         nullptr,
//                                         new ceres::CauchyLoss(0.5),
//                                         new ceres::HuberLoss(1.0),
                                         point.data(),
                                         pose.data(),
                                         orientation.data(),
                                         intr.data());
                problem.SetParameterization(orientation.data(), quaternionLocalParameterization);
                problem.SetParameterBlockConstant(intr.data());
            }
        }
        problem.SetParameterBlockConstant(poseTxTyTzByPoseNumber[indexFixed].data());
        problem.SetParameterBlockConstant(orientationsqxyzwByPoseNumber[indexFixed].data());


        ceres::Solver::Options options;
//        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 500;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << "done ceres BA" << std::endl;
        std::cout << summary.FullReport() << std::endl;

        // check if solution can be used
        std::cout << "Is BA USABLE?" << std::endl;
        assert(summary.IsSolutionUsable());
        std::cout << "Threads used " << summary.num_threads_used << std::endl;

        std::cout << "BA usable" << std::endl;

        std::vector<Sophus::SE3d> posesOptimized;

        assert(cameraModelByPoseNumber.size() == poseTxTyTzByPoseNumber.size());
        for (int i = 0; i < poseTxTyTzByPoseNumber.size(); ++i) {
            Eigen::Quaterniond orientation(orientationsqxyzwByPoseNumber[i].data());
            const auto &poseTranslationCameraIntr = poseTxTyTzByPoseNumber[i];
            std::vector<double> t = {poseTranslationCameraIntr[0], poseTranslationCameraIntr[1],
                                     poseTranslationCameraIntr[2]};
            Eigen::Vector3d translation(t.data());
//            std::cout << "orientation " << orientation.norm() << std::endl;
            assert(std::abs(orientation.norm() - 1.0) < 1e-10);
            Sophus::SE3d poseCurrent;
            poseCurrent.setQuaternion(orientation);
            poseCurrent.translation() = translation;
            posesOptimized.push_back(poseCurrent);
            const auto &camera = cameraModelByPoseNumber[i];
            const auto &intr = poseFxCxFyCyScaleByPoseNumber[i];
            double eps = 30 * std::numeric_limits<double>::epsilon();
            double errorFx = std::abs(camera.fx - intr[cameraIntrStartIndex]);
            double errorCx = std::abs(camera.cx - intr[cameraIntrStartIndex + 1]);
            double errorFy = std::abs(camera.fy - intr[cameraIntrStartIndex + 2]);
            double errorCy = std::abs(camera.cy - intr[cameraIntrStartIndex + 3]);
            assert(errorFx < eps);
            assert(errorCx < eps);
            assert(errorCy < eps);
            assert(errorFy < eps);


        }
//        for (const auto& pose: poseTxTyTzByPoseNumber) {
//            Eigen::Quaterniond orientation(&pose[quatStartIndex]);
//            Eigen::Vector3d translation(&pose[0]);
//            std::cout << "orientation " << orientation.norm() << std::endl;
////            assert(std::abs(orientation.norm() - 1.0) < 1e-10);
//            Sophus::SE3d poseCurrent;
//            poseCurrent.setQuaternion(orientation);
//            poseCurrent.translation() = translation;
//            posesOptimized.push_back(poseCurrent);
//
//        }

        return posesOptimized;
    }

    std::vector<Point3d> BundleAdjuster::getPointsGlobalCoordinatesOptimized() const {
        std::vector<Point3d> pointsOtimized;

        for (int i = 0; i < pointsXYZbyIndex.size(); ++i) {
            const auto &point = pointsXYZbyIndex[i];
            assert(point.size() == 3);
            pointsOtimized.push_back(Point3d(point[0], point[1], point[2], i));
        }
        assert(pointsOtimized.size() == pointsXYZbyIndex.size());
        return pointsOtimized;
    }

    // get median and max errors: reprojection OX and OY and Depth: [{OX, OY, Depth}_median, {OX, OY, Depth}_max, {OX, OY, Depth}_min]

    std::vector<double> BundleAdjuster::getMedianErrorsXYDepth() {

        std::vector<double> errorsReprojectionX;
        std::vector<double> errorsReprojectionY;
        std::vector<double> errorsDepth;

        double minScale = std::numeric_limits<double>::infinity();
        double maxScale = -1;

        for (int currPose = 0; currPose < poseTxTyTzByPoseNumber.size(); ++currPose) {
            for (const auto &keyPointInfosByPose: keyPointInfoByPoseNumberAndPointNumber[currPose]) {

                int currPoint = keyPointInfosByPose.first;
                const auto &camera = cameraModelByPoseNumber[currPose];
                const auto poseTransformation = getSE3TransformationMatrixByPoseNumber(currPose);
                const auto point3d = getPointVector4dByPointGlobalIndex(currPoint);
                Eigen::Vector3d localCameraCoordinatesOfPoint = poseTransformation.inverse().matrix3x4() * point3d;
                Eigen::Vector3d imageCoordinates = camera.getIntrinsicsMatrix3x3() * localCameraCoordinatesOfPoint;
                double computedX = imageCoordinates[0] / imageCoordinates[2];
                double computedY = imageCoordinates[1] / imageCoordinates[2];
                double computedDepth = localCameraCoordinatesOfPoint[2];

                const auto &keyPointInfo = keyPointInfoByPoseNumberAndPointNumber[currPose][currPoint];

                minScale = std::min(minScale, keyPointInfo.getScale());
                maxScale = std::max(maxScale, keyPointInfo.getScale());
                assert(keyPointInfo.isInitialized());
                errorsReprojectionX.push_back(std::abs(computedX - keyPointInfo.getX()));
                errorsReprojectionY.push_back(std::abs(computedY - keyPointInfo.getY()));
                errorsDepth.push_back(std::abs(computedDepth - keyPointInfo.getDepth()));
            }
        }
        std::cout << "min max scale: " << minScale << ' ' << maxScale << std::endl;
        assert(errorsDepth.size() == errorsReprojectionX.size());
        assert(errorsDepth.size() == errorsReprojectionY.size());

        int indexMedianOfMeasurements = errorsReprojectionX.size() / 2;

        std::nth_element(errorsReprojectionX.begin(), errorsReprojectionX.begin() + indexMedianOfMeasurements,
                         errorsReprojectionX.end());
        std::nth_element(errorsReprojectionY.begin(), errorsReprojectionY.begin() + indexMedianOfMeasurements,
                         errorsReprojectionY.end());
        std::nth_element(errorsDepth.begin(), errorsDepth.begin() + indexMedianOfMeasurements, errorsDepth.end());

        double medianErrorX = errorsReprojectionX[indexMedianOfMeasurements];
        double medianErrorY = errorsReprojectionY[indexMedianOfMeasurements];
        double medianErrorDepth = errorsDepth[indexMedianOfMeasurements];


        int indexQuantile90OfMeasurements = errorsReprojectionX.size() * 0.9;

        std::nth_element(errorsReprojectionX.begin(), errorsReprojectionX.begin() + indexQuantile90OfMeasurements,
                         errorsReprojectionX.end());
        std::nth_element(errorsReprojectionY.begin(), errorsReprojectionY.begin() + indexQuantile90OfMeasurements,
                         errorsReprojectionY.end());
        std::nth_element(errorsDepth.begin(), errorsDepth.begin() + indexQuantile90OfMeasurements, errorsDepth.end());

        double medianErrorXquant90 = errorsReprojectionX[indexQuantile90OfMeasurements];
        double medianErrorYquant90 = errorsReprojectionY[indexQuantile90OfMeasurements];
        double medianErrorDepthquant90 = errorsDepth[indexQuantile90OfMeasurements];

        return {medianErrorX, medianErrorY, medianErrorDepth, medianErrorXquant90, medianErrorYquant90,
                medianErrorDepthquant90};
    }

    // each unordered map maps from poseNumber to unordered map
    // mapping from keyPointGlobalIndex to {errorPixelX, errorPixelY}
    std::vector<Sophus::SE3d> BundleAdjuster::optimizePointsAndPosesUsingDepthInfo(int indexFixed) {


        std::cout << "entered BA depth optimization" << std::endl;

        std::vector<double> medians = getMedianErrorsXYDepth();

        double medianErrorX = medians[0];
        double medianErrorY = medians[1];
        double medianErrorDepth = medians[2];

        double quantile90ErrorX = medians[3];
        double quantile90ErrorY = medians[4];
        double quantile90ErrorDepth = medians[5];

        ceres::Problem problem;
        ceres::LocalParameterization *quaternionLocalParameterization =
                new ceres::EigenQuaternionParameterization;
        std::cout << "started BA [depth using] ! " << std::endl;
        assert(orientationsqxyzwByPoseNumber.size() == poseTxTyTzByPoseNumber.size());
        assert(!orientationsqxyzwByPoseNumber.empty());
        for (int poseIndex = 0; poseIndex < poseTxTyTzByPoseNumber.size(); ++poseIndex) {

//            std::cout << "init pose " << poseIndex << std::endl;
            auto &pose = poseTxTyTzByPoseNumber[poseIndex];
            auto &orientation = orientationsqxyzwByPoseNumber[poseIndex];

            assert(poseIndex < keyPointInfoByPoseNumberAndPointNumber.size());

            for (const auto &indexAndKeyPointInfo: keyPointInfoByPoseNumberAndPointNumber[poseIndex]) {
                int pointIndex = indexAndKeyPointInfo.first;
                assert(pointIndex >= 0 && pointIndex < pointsXYZbyIndex.size());

                auto &point = pointsXYZbyIndex[pointIndex];
//                auto& observedKeyPoints = keyPointInfoByPoseNumberAndPointNumber[poseIndex];
                const auto &keyPointInfo = indexAndKeyPointInfo.second;

                assert(keyPointInfo.isInitialized());
                double widthAssert = 640;
                double heightAssert = 480;
                double observedX = keyPointInfo.getX();
                double observedY = keyPointInfo.getY();

                if (!(observedX > 0 && observedX < widthAssert)) {
                    std::cout << "observed x: " << observedX << std::endl;
                }
                if (!(observedY > 0 && observedY < heightAssert)) {
                    std::cout << "observed y: " << observedY << std::endl;
                }
                assert(observedX > 0 && observedX < widthAssert);
                assert(observedY > 0 && observedY < heightAssert);

                ceres::CostFunction *cost_function =
                        ReprojectionWithDepthError::Create(observedX, observedY, keyPointInfo.getDepth(),
                                                           keyPointInfo.getScale(),
                                                           cameraModelByPoseNumber[poseIndex],
                                                           medianErrorX, medianErrorY, medianErrorDepth);
                // no robustness at the moment
                problem.AddResidualBlock(cost_function,
//                                         nullptr,
//                                         new ceres::CauchyLoss(0.5),
                                         new ceres::HuberLoss(2.0),
                                         point.data(),
                                         pose.data(),
                                         orientation.data());
                problem.SetParameterization(orientation.data(), quaternionLocalParameterization);
            }
        }
        problem.SetParameterBlockConstant(poseTxTyTzByPoseNumber[indexFixed].data());
        problem.SetParameterBlockConstant(orientationsqxyzwByPoseNumber[indexFixed].data());


        ceres::Solver::Options options;
//        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
//        options.num_threads = 6;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << "done ceres BA" << std::endl;
        std::cout << summary.FullReport() << std::endl;

        // check if solution can be used
        std::cout << "Is BA USABLE?" << std::endl;
        assert(summary.IsSolutionUsable());
        std::cout << "Threads used " << summary.num_threads_used << std::endl;

        std::cout << "BA usable" << std::endl;

        std::vector<double> mediansAfter = getMedianErrorsXYDepth();

        double medianErrorXafter = mediansAfter[0];
        double medianErrorYafter = mediansAfter[1];
        double medianErrorDepthAfter = mediansAfter[2];

        double quantile90ErrorXafter = mediansAfter[3];
        double quantile90ErrorYafter = mediansAfter[4];
        double quantile90ErrorDepthAfter = mediansAfter[5];

        std::cout << "=============================median errors information!============================" << std::endl;
        std::cout << "medians BEFORE (x, y, depth), quantiles (x, y, depth): "
                  << medianErrorX << ", " << medianErrorY << ", " << medianErrorDepth << ", "
                  << quantile90ErrorX << ", " << quantile90ErrorY << ", " << quantile90ErrorDepth
                  << std::endl;
        std::cout << "medians AFTER (x, y, depth), quantiles (x, y, depth): "
                  << medianErrorXafter << ", " << medianErrorYafter << ", " << medianErrorDepthAfter << ", "
                  << quantile90ErrorXafter << ", " << quantile90ErrorYafter << ", " << quantile90ErrorDepthAfter
                  << std::endl;
        std::vector<Sophus::SE3d> posesOptimized;

        assert(cameraModelByPoseNumber.size() == poseTxTyTzByPoseNumber.size());
        for (int i = 0; i < poseTxTyTzByPoseNumber.size(); ++i) {
            Eigen::Quaterniond orientation(orientationsqxyzwByPoseNumber[i].data());
            const auto &poseTranslationCameraIntr = poseTxTyTzByPoseNumber[i];
            std::vector<double> t = {poseTranslationCameraIntr[0], poseTranslationCameraIntr[1],
                                     poseTranslationCameraIntr[2]};
            Eigen::Vector3d translation(t.data());
//            std::cout << "orientation " << orientation.norm() << std::endl;
            assert(std::abs(orientation.norm() - 1.0) < 1e-10);
            Sophus::SE3d poseCurrent;
            poseCurrent.setQuaternion(orientation);
            poseCurrent.translation() = translation;
            posesOptimized.push_back(poseCurrent);
            const auto &camera = cameraModelByPoseNumber[i];
            const auto &intr = poseFxCxFyCyScaleByPoseNumber[i];
            double eps = 30 * std::numeric_limits<double>::epsilon();
            double errorFx = std::abs(camera.fx - intr[cameraIntrStartIndex]);
            double errorCx = std::abs(camera.cx - intr[cameraIntrStartIndex + 1]);
            double errorFy = std::abs(camera.fy - intr[cameraIntrStartIndex + 2]);
            double errorCy = std::abs(camera.cy - intr[cameraIntrStartIndex + 3]);
            assert(errorFx < eps);
            assert(errorCx < eps);
            assert(errorCy < eps);
            assert(errorFy < eps);

        }

        return posesOptimized;
    }

    Sophus::SE3d BundleAdjuster::getSE3TransformationMatrixByPoseNumber(int poseNumber) const {

        Sophus::SE3d pose;
        pose.setQuaternion(Eigen::Quaterniond(orientationsqxyzwByPoseNumber[poseNumber].data()));
        pose.translation() = Eigen::Vector3d(poseTxTyTzByPoseNumber[poseNumber].data());

        return pose;
    }

    Eigen::Vector3d BundleAdjuster::getPointVector3dByPointGlobalIndex(int pointGlobalIndex) const {
        return Eigen::Vector3d(pointsXYZbyIndex[pointGlobalIndex].data());
    }

    Eigen::Vector4d BundleAdjuster::getPointVector4dByPointGlobalIndex(int pointGlobalIndex) const {
        Eigen::Vector4d point4d;
        point4d.setOnes();
        for (int i = 0; i < 3; ++i) {
            point4d[i] = pointsXYZbyIndex[pointGlobalIndex][i];
        }
        return point4d;
    }
}
