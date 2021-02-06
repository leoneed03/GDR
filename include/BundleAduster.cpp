//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "BundleAduster.h"

#include <ceres/ceres.h>

namespace gdr {

    BundleAdjuster::BundleAdjuster(const std::vector<Point3d> &points,
                                   const std::vector<std::pair<Sophus::SE3d, CameraRGBD>> &absolutePoses,
                                   const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointinfo) {
        assert(keyPointinfo.size() == absolutePoses.size());
        assert(!absolutePoses.empty());
        assert(absolutePoses.size() > 0);
        std::cout << "poses: " << absolutePoses.size() << std::endl;
        for (const auto &point: points) {
            pointsXYZbyIndex.push_back(point.getVectorPointXYZ());
        }
        keyPointInfoByPoseNumberAndPointNumber = keyPointinfo;

        for (const auto &pose: absolutePoses) {
            const auto &translation = pose.first.translation();
            const auto &rotationQuat = pose.first.unit_quaternion();
            const auto &cameraIntr = pose.second;
            poseXYZqxyzwByPoseNumber.push_back({translation[0], translation[1], translation[2]});
            poseFxCxFyCyByPoseNumber.push_back({cameraIntr.fx, cameraIntr.cx, cameraIntr.fy, cameraIntr.cy});
            orientationsqxyzwByPoseNumber.push_back({rotationQuat.x(), rotationQuat.y(), rotationQuat.z(), rotationQuat.w()});

            cameraModelByPoseNumber.push_back(pose.second);
            assert(poseXYZqxyzwByPoseNumber[poseXYZqxyzwByPoseNumber.size() - 1].size() == dimPose);
        }

        assert(pointsXYZbyIndex.size() == points.size());
        assert(keyPointinfo.size() == keyPointInfoByPoseNumberAndPointNumber.size());
        assert(absolutePoses.size() == poseXYZqxyzwByPoseNumber.size());

    }

    std::vector<Sophus::SE3d> BundleAdjuster::optimizePointsAndPoses(int indexFixed) {


        ceres::Problem problem;
        ceres::LocalParameterization *quaternionLocalParameterization =
                new ceres::EigenQuaternionParameterization;
        std::cout << "started BA" << std::endl;

        assert(orientationsqxyzwByPoseNumber.size() == poseXYZqxyzwByPoseNumber.size());
        assert(!orientationsqxyzwByPoseNumber.empty());
        for (int poseIndex = 0; poseIndex < poseXYZqxyzwByPoseNumber.size(); ++poseIndex) {

            std::cout << "init pose " << poseIndex << std::endl;
            auto &pose = poseXYZqxyzwByPoseNumber[poseIndex];
            auto &orientation = orientationsqxyzwByPoseNumber[poseIndex];

            assert(poseIndex < keyPointInfoByPoseNumberAndPointNumber.size());

            for (const auto& indexAndKeyPointInfo: keyPointInfoByPoseNumberAndPointNumber[poseIndex]) {
                int pointIndex = indexAndKeyPointInfo.first;
                assert(pointIndex >= 0 && pointIndex < pointsXYZbyIndex.size());

                auto &point = pointsXYZbyIndex[pointIndex];
                auto &intr = poseFxCxFyCyByPoseNumber[poseIndex];
                assert(intr.size() == cameraDim);
                auto& observedKeyPoints = keyPointInfoByPoseNumberAndPointNumber[poseIndex];
                const auto& keyPointInfo = indexAndKeyPointInfo.second;

                double width = 640;
                double height = 480;
                double observedX = width - keyPointInfo.getX();
                double observedY = height - keyPointInfo.getY();

                assert(observedX > 0 && observedX < width);
                assert(observedY > 0 && observedY < height);

                ceres::CostFunction *cost_function =
                        ReprojectionError::Create(observedX, observedY, keyPointInfo.getDepth());
                problem.AddResidualBlock(cost_function,
                                         new ceres::CauchyLoss(0.5),
                                         point.data(),
                                         pose.data(),
                                         orientation.data(),
                                         intr.data());
                problem.SetParameterization(orientation.data(), quaternionLocalParameterization);
                problem.SetParameterBlockConstant(intr.data());
            }
        }
        problem.SetParameterBlockConstant(poseXYZqxyzwByPoseNumber[indexFixed].data());
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

        assert(cameraModelByPoseNumber.size() == poseXYZqxyzwByPoseNumber.size());
        for (int i = 0; i < poseXYZqxyzwByPoseNumber.size(); ++i) {
            Eigen::Quaterniond orientation(orientationsqxyzwByPoseNumber[i].data());
            const auto& poseTranslationCameraIntr = poseXYZqxyzwByPoseNumber[i];
            std::vector<double> t = {poseTranslationCameraIntr[0], poseTranslationCameraIntr[1], poseTranslationCameraIntr[2]};
            Eigen::Vector3d translation(t.data());
//            std::cout << "orientation " << orientation.norm() << std::endl;
            assert(std::abs(orientation.norm() - 1.0) < 1e-10);
            Sophus::SE3d poseCurrent;
            poseCurrent.setQuaternion(orientation);
            poseCurrent.translation() = translation;
            posesOptimized.push_back(poseCurrent);
            const auto& camera = cameraModelByPoseNumber[i];
            const auto& intr = poseFxCxFyCyByPoseNumber[i];
            double eps = 30 * std::numeric_limits<double>::epsilon();
            double errorFx = std::abs(camera.fx - intr[cameraIntrStartIndex]);
            double errorCx = std::abs(camera.cx - intr[cameraIntrStartIndex + 1]);
            double errorFy = std::abs(camera.fy - intr[cameraIntrStartIndex + 2]);
            double errorCy = std::abs(camera.cy - intr[cameraIntrStartIndex + 3]);
            std::cout << camera.fx << " vs (camera.fx) " << intr[cameraIntrStartIndex] << " with error " << errorFx << std::endl;
            std::cout << camera.cx << " vs (camera.cx) " << intr[cameraIntrStartIndex + 1] << " with error " << errorCx << std::endl;
            std::cout << camera.fy << " vs (camera.fy) " << intr[cameraIntrStartIndex + 2] << " with error " << errorFy << std::endl;
            std::cout << camera.cy << " vs (camera.cy) " << intr[cameraIntrStartIndex + 3] << " with error " << errorCy << std::endl;
            assert(errorFx < eps);
            assert(errorCx < eps);
            assert(errorCy < eps);
            assert(errorFy < eps);
            std::cout << "camera params constant -- ok" << std::endl;


        }
//        for (const auto& pose: poseXYZqxyzwByPoseNumber) {
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
            const auto& point = pointsXYZbyIndex[i];
            assert(point.size() == 3);
            pointsOtimized.push_back(Point3d(point[0], point[1], point[2], i));
        }
        assert(pointsOtimized.size() == pointsXYZbyIndex.size());
        return pointsOtimized;
    }
}
