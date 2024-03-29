//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "bundleAdjustment/BundleDepthAdjuster.h"

#include <ceres/ceres.h>

namespace gdr {

    std::pair<std::vector<double>, std::vector<double>>
    BundleDepthAdjuster::getNormalizedErrorsReprojectionAndDepth(bool performNormalizing) {
        std::vector<double> errorsReprojectionXY;
        std::vector<double> errorsDepth;

        double minScale = std::numeric_limits<double>::infinity();
        double maxScale = -1;

        for (int currPose = 0; currPose < poseWorldToCameraTxTyTzByPoseNumber.size(); ++currPose) {
            for (const auto &keyPointInfosByPose: keyPointInfoByPoseNumberAndPointNumber[currPose]) {

                int currPoint = keyPointInfosByPose.first;
                const auto &camera = cameraModelByPoseNumber[currPose];
                const auto poseTransformation = getSE3TransformationMatrixByPoseNumber(currPose);
                const auto point3d = getPointVector3dByPointGlobalIndex(currPoint);

                Sophus::Vector3d localCameraCoordinatesOfPoint = poseTransformation * point3d;

                Sophus::Vector<double, 2> imageCoordinatesNormalized =
                        camera.projectUsingIntrinsics<double>(
                                localCameraCoordinatesOfPoint);

                double computedX = imageCoordinatesNormalized[0];
                double computedY = imageCoordinatesNormalized[1];

                double computedDepth = localCameraCoordinatesOfPoint[2];

                const auto &keyPointInfo = keyPointInfoByPoseNumberAndPointNumber[currPose][currPoint];

                minScale = std::min(minScale, keyPointInfo.getScale());
                maxScale = std::max(maxScale, keyPointInfo.getScale());
                assert(keyPointInfo.isInitialized());

                double errorX = std::abs(computedX - keyPointInfo.getX());
                double errorY = std::abs(computedY - keyPointInfo.getY());

                const auto &measurementEstimators = camera.getMeasurementErrorDeviationEstimators();

                Sophus::Vector2d errorReproj(errorX, errorY);

                const auto &dividerReproj = camera.getMeasurementErrorDeviationEstimators().getDividerReprojectionEstimator();
                double normalizedReprojError = errorReproj.lpNorm<2>() /
                                               (dividerReproj)(keyPointInfo.getScale(),
                                                               measurementEstimators.getParameterNoiseModelReprojection());

                double rawReprojError = errorReproj.lpNorm<2>();
                double reprojErrorToUse = performNormalizing ? normalizedReprojError : rawReprojError;

                errorsReprojectionXY.emplace_back(reprojErrorToUse);

                double depthError = std::abs(computedDepth - keyPointInfo.getDepth());
                const auto &dividerDepth = camera.getMeasurementErrorDeviationEstimators().getDividerDepthErrorEstimator();
                double normalizedErrorDepth = depthError /
                                              (dividerDepth)(keyPointInfo.getDepth(),
                                                             measurementEstimators.getParameterNoiseModelDepth());
                double depthErrorToUse = performNormalizing ? normalizedErrorDepth : depthError;

                errorsDepth.emplace_back(depthErrorToUse);
            }
        }

        assert(errorsReprojectionXY.size() == errorsDepth.size());
        assert(!errorsReprojectionXY.empty());

        return {errorsReprojectionXY, errorsDepth};
    }

    std::vector<Point3d> BundleDepthAdjuster::getOptimizedPoints() const {
        std::vector<Point3d> pointsOptimized;

        for (int i = 0; i < pointsXYZbyIndex.size(); ++i) {
            const auto &point = pointsXYZbyIndex[i];
            assert(point.size() == 3);
            pointsOptimized.emplace_back(Point3d(point[0], point[1], point[2], i));
        }

        assert(pointsOptimized.size() == pointsXYZbyIndex.size());
        return pointsOptimized;
    }

    /** [DEBUG] Get median and 0.9 qunatile errors: reprojection OX and OY and Depth:
     *      [{OX, OY, Depth}_median, {OX, OY, Depth}_max, {OX, OY, Depth}_min]
     */
    std::vector<double> BundleDepthAdjuster::getMedianErrorsXYDepth() {

        std::vector<double> errorsReprojectionX;
        std::vector<double> errorsReprojectionY;
        std::vector<double> errorsDepth;

        double minScale = std::numeric_limits<double>::infinity();
        double maxScale = -1;

        for (int currPose = 0; currPose < poseWorldToCameraTxTyTzByPoseNumber.size(); ++currPose) {
            for (const auto &keyPointInfosByPose: keyPointInfoByPoseNumberAndPointNumber[currPose]) {

                int currPoint = keyPointInfosByPose.first;
                const auto &camera = cameraModelByPoseNumber[currPose];

                const auto poseTransformation = getSE3TransformationMatrixByPoseNumber(currPose);
                const auto point3d = getPointVector3dByPointGlobalIndex(currPoint);

                Eigen::Vector3d localCameraCoordinatesOfPoint = poseTransformation * point3d;
                Sophus::Vector2d imageCoordinatesNormalized =
                        camera.projectUsingIntrinsics<double>(localCameraCoordinatesOfPoint);

                double computedX = imageCoordinatesNormalized[0];
                double computedY = imageCoordinatesNormalized[1];
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

    std::vector<SE3> BundleDepthAdjuster::optimizePointsAndPoses(const std::vector<Point3d> &points,
                                                                 const std::vector<std::pair<SE3, CameraRGBD>> &posesCameraToWorld,
                                                                 const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfos,
                                                                 int indexFixed,
                                                                 bool &success) {

        setPosesAndPoints(points, posesCameraToWorld, keyPointInfos);

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

        assert(orientationWorldToCameraqxyzwByPoseNumber.size() == poseWorldToCameraTxTyTzByPoseNumber.size());
        assert(!orientationWorldToCameraqxyzwByPoseNumber.empty());

        std::pair<double, double> deviationEstimatorsSigmaReprojAndDepth = getSigmaReprojectionAndDepth();
        double sigmaReproj = deviationEstimatorsSigmaReprojAndDepth.first;
        double sigmaDepth = deviationEstimatorsSigmaReprojAndDepth.second;

        //errors without "normalizing" i.e. dividing by deviation estimation sigma_ij
        std::pair<std::vector<double>, std::vector<double>> errorsReprojDepthRaw =
                getNormalizedErrorsReprojectionAndDepth(false);

        double medianErrorReprojRaw = RobustEstimators::getQuantile(errorsReprojDepthRaw.first);
        double medianErrorDepthRaw = RobustEstimators::getQuantile(errorsReprojDepthRaw.second);
        auto errors3DL2Raw = getL2Errors();
        double medianErrorL2Raw = RobustEstimators::getQuantile(errors3DL2Raw);

        assert(errors3DL2Raw.size() == errorsReprojDepthRaw.first.size());

        for (int poseIndex = 0; poseIndex < poseWorldToCameraTxTyTzByPoseNumber.size(); ++poseIndex) {

            auto &pose = poseWorldToCameraTxTyTzByPoseNumber[poseIndex];
            auto &orientation = orientationWorldToCameraqxyzwByPoseNumber[poseIndex];

            assert(poseIndex < keyPointInfoByPoseNumberAndPointNumber.size());

            for (const auto &indexAndKeyPointInfo: keyPointInfoByPoseNumberAndPointNumber[poseIndex]) {
                int pointIndex = indexAndKeyPointInfo.first;
                assert(pointIndex >= 0 && pointIndex < pointsXYZbyIndex.size());

                auto &point = pointsXYZbyIndex[pointIndex];
                const auto &keyPointInfo = indexAndKeyPointInfo.second;

                assert(keyPointInfo.isInitialized());
                double widthAssert = 640;
                double heightAssert = 480;
                double observedX = keyPointInfo.getX();
                double observedY = keyPointInfo.getY();

                assert(observedX > 0 && observedX < widthAssert);
                assert(observedY > 0 && observedY < heightAssert);

                const auto &camera = cameraModelByPoseNumber[poseIndex];

                const auto &measurementEstimators = camera.getMeasurementErrorDeviationEstimators();
                const auto &dividerReproj = measurementEstimators.getDividerReprojectionEstimator();

                double deviationEstReprojByScale = (dividerReproj)(keyPointInfo.getScale(),
                                                                   measurementEstimators.getParameterNoiseModelReprojection());


                const auto &dividerDepth = measurementEstimators.getDividerDepthErrorEstimator();
                double deviationEstDepthByDepth = (dividerDepth)(keyPointInfo.getDepth(),
                                                                 measurementEstimators.getParameterNoiseModelDepth());

                ceres::CostFunction *cost_function_depth =
                        DepthOnlyResidual::Create(keyPointInfo.getDepth(),
                                                  cameraModelByPoseNumber[poseIndex],
                                                  sigmaDepth,
                                                  deviationEstDepthByDepth,
                                                  medianErrorDepthRaw);
                ceres::CostFunction *cost_function_reproj =
                        ReprojectionOnlyResidual::Create(observedX,
                                                         observedY,
                                                         keyPointInfo.getScale(),
                                                         cameraModelByPoseNumber[poseIndex],
                                                         sigmaReproj,
                                                         deviationEstReprojByScale,
                                                         medianErrorReprojRaw);
                problem.AddResidualBlock(cost_function_reproj,
                                         new ceres::CauchyLoss(sigmaReproj),
                                         point.data(),
                                         pose.data(),
                                         orientation.data());

                problem.AddResidualBlock(cost_function_depth,
                                         new ceres::CauchyLoss(sigmaDepth),
                                         point.data(),
                                         pose.data(),
                                         orientation.data());

                problem.SetParameterization(orientation.data(), quaternionLocalParameterization);
            }
        }
        problem.SetParameterBlockConstant(poseWorldToCameraTxTyTzByPoseNumber[indexFixed].data());
        problem.SetParameterBlockConstant(orientationWorldToCameraqxyzwByPoseNumber[indexFixed].data());


        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = getMaxNumberIterations();
        options.num_threads = getMaxNumberThreads();

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        assert(summary.IsSolutionUsable() && "ceres marked solution as unusable");

        std::pair<std::vector<double>, std::vector<double>> errorsReprojDepthRawAfter =
                getNormalizedErrorsReprojectionAndDepth(false);

        double medianErrorReprojRawAfter = RobustEstimators::getQuantile(errorsReprojDepthRawAfter.first);
        double medianErrorDepthRawAfter = RobustEstimators::getQuantile(errorsReprojDepthRawAfter.second);
        double medianErrorL2RawAfter = RobustEstimators::getQuantile(getL2Errors());

        std::vector<double> mediansAfter = getMedianErrorsXYDepth();

        double medianErrorXafter = mediansAfter[0];
        double medianErrorYafter = mediansAfter[1];
        double medianErrorDepthAfter = mediansAfter[2];

        double quantile90ErrorXafter = mediansAfter[3];
        double quantile90ErrorYafter = mediansAfter[4];
        double quantile90ErrorDepthAfter = mediansAfter[5];

        std::vector<SE3> posesOptimized;

        assert(cameraModelByPoseNumber.size() == poseWorldToCameraTxTyTzByPoseNumber.size());

        for (int i = 0; i < poseWorldToCameraTxTyTzByPoseNumber.size(); ++i) {

            Eigen::Quaterniond orientation(orientationWorldToCameraqxyzwByPoseNumber[i].data());
            const auto &poseTranslationCameraIntr = poseWorldToCameraTxTyTzByPoseNumber[i];

            std::vector<double> t = {poseTranslationCameraIntr[0], poseTranslationCameraIntr[1],
                                     poseTranslationCameraIntr[2]};
            Eigen::Vector3d translation(t.data());
            Sophus::SE3d poseCurrent;
            poseCurrent.setQuaternion(orientation);
            poseCurrent.translation() = translation;

            posesOptimized.emplace_back(SE3(poseCurrent).inverse());
        }

        return posesOptimized;
    }

    Sophus::SE3d BundleDepthAdjuster::getSE3TransformationMatrixByPoseNumber(int poseNumber) const {

        Sophus::SE3d pose;
        pose.setQuaternion(Eigen::Quaterniond(orientationWorldToCameraqxyzwByPoseNumber[poseNumber].data()));
        pose.translation() = Eigen::Vector3d(poseWorldToCameraTxTyTzByPoseNumber[poseNumber].data());

        return pose;
    }

    Eigen::Vector3d BundleDepthAdjuster::getPointVector3dByPointGlobalIndex(int pointGlobalIndex) const {
        return Eigen::Vector3d(pointsXYZbyIndex[pointGlobalIndex].data());
    }

    Eigen::Vector4d BundleDepthAdjuster::getPointVector4dByPointGlobalIndex(int pointGlobalIndex) const {
        Eigen::Vector4d point4d;
        point4d.setOnes();
        for (int i = 0; i < 3; ++i) {
            point4d[i] = pointsXYZbyIndex[pointGlobalIndex][i];
        }
        return point4d;
    }

    std::vector<double> BundleDepthAdjuster::getInlierErrors(const std::vector<double> &r_n,
                                                             double s_0,
                                                             double thresholdInlier) {
        std::vector<double> inliers;

        assert(!r_n.empty());
        assert(s_0 > 0);
        assert(thresholdInlier > 0);

        for (const auto &r_i: r_n) {
            if (std::abs(1.0 * r_i / s_0) < thresholdInlier) {
                inliers.emplace_back(r_i);
            }
        }
        assert(!inliers.empty());

        return inliers;
    }

    std::pair<std::vector<double>, std::vector<double>>
    BundleDepthAdjuster::getInlierNormalizedErrorsReprojectionAndDepth(double thresholdInlier) {

        std::vector<double> inlierErrorsReprojection;
        std::vector<double> inlierErrorsDepth;

        std::pair<std::vector<double>, std::vector<double>> normalizedErrorsReprojAndDepth =
                getNormalizedErrorsReprojectionAndDepth();

        const auto &errorsNormalizedReproj = normalizedErrorsReprojAndDepth.first;
        const auto &errorsNormalizedDepth = normalizedErrorsReprojAndDepth.second;

        assert(!errorsNormalizedReproj.empty());
        assert(errorsNormalizedReproj.size() == errorsNormalizedDepth.size());

        double medianNormalizedReprojection = RobustEstimators::getQuantile(errorsNormalizedReproj);
        double medianNormalizedDepth = RobustEstimators::getQuantile(errorsNormalizedDepth);

        double initScaleReproj = computeInitialScaleByMedian(medianNormalizedReprojection);
        double initScaleDepth = computeInitialScaleByMedian(medianNormalizedDepth);

        inlierErrorsReprojection = getInlierErrors(errorsNormalizedReproj, initScaleReproj, thresholdInlier);
        inlierErrorsDepth = getInlierErrors(errorsNormalizedDepth, initScaleDepth, thresholdInlier);

        assert(!inlierErrorsReprojection.empty());
        assert(!inlierErrorsDepth.empty());

        return {inlierErrorsReprojection, inlierErrorsDepth};

    }

    double getFinalScaleEstimate(const std::vector<double> &inlierErrors,
                                 int p) {

        assert(p == 0);
        double sumErrorSquared = 0.0;
        for (const auto &error: inlierErrors) {
            sumErrorSquared += std::pow(error, 2.0);
        }
        sumErrorSquared /= static_cast<double>(inlierErrors.size() - p);

        return std::sqrt(sumErrorSquared);
    }

    std::pair<double, double> BundleDepthAdjuster::getSigmaReprojectionAndDepth(double threshold) {
        int pToSubstract = (p > 0) ? (p) : (0);

        std::pair<std::vector<double>, std::vector<double>> inlierErrorsReprojAndDepth =
                getInlierNormalizedErrorsReprojectionAndDepth(threshold);

        const auto &errorsReproj = inlierErrorsReprojAndDepth.first;
        const auto &errorsDepth = inlierErrorsReprojAndDepth.second;

        double sigmaReproj = getFinalScaleEstimate(errorsReproj, pToSubstract);
        double sigmaDepth = getFinalScaleEstimate(errorsDepth, pToSubstract);

        return {sigmaReproj, sigmaDepth};
    }

    std::vector<double> BundleDepthAdjuster::getL2Errors() {
        std::vector<double> errorsL2;

        for (int currPose = 0; currPose < poseWorldToCameraTxTyTzByPoseNumber.size(); ++currPose) {
            for (const auto &keyPointInfosByPose: keyPointInfoByPoseNumberAndPointNumber[currPose]) {

                int currPoint = keyPointInfosByPose.first;
                const auto &keyPointInfo = keyPointInfoByPoseNumberAndPointNumber[currPose][currPoint];

                const auto &camera = cameraModelByPoseNumber[currPose];
                const auto poseTransformation = getSE3TransformationMatrixByPoseNumber(currPose);
                const auto point3d = getPointVector3dByPointGlobalIndex(currPoint);
                const auto observedPoint3d = camera.getCoordinates3D(keyPointInfo.getX(),
                                                                     keyPointInfo.getY(),
                                                                     keyPointInfo.getDepth());

                Eigen::Vector3d localCameraCoordinatesOfPoint = poseTransformation * point3d;
                auto difference3D = localCameraCoordinatesOfPoint - observedPoint3d;

                errorsL2.emplace_back(difference3D.norm());
            }
        }

        assert(!errorsL2.empty());

        return errorsL2;
    }

    void BundleDepthAdjuster::setPosesAndPoints(const std::vector<Point3d> &points,
                                                const std::vector<std::pair<SE3, CameraRGBD>> &posesCameraToWorld,
                                                const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfo) {

        assert(keyPointInfo.size() == posesCameraToWorld.size());
        assert(!posesCameraToWorld.empty());

        for (const auto &point: points) {
            pointsXYZbyIndex.emplace_back(point.getVectorPointXYZ());
        }

        for (const auto &mapIntInfo: keyPointInfo) {
            for (const auto &pairIntInfo: mapIntInfo) {
                assert(pairIntInfo.second.getX() >= 0);
                assert(pairIntInfo.second.getY() >= 0);
            }
        }
        keyPointInfoByPoseNumberAndPointNumber = keyPointInfo;

        for (const auto &pose: posesCameraToWorld) {

            Sophus::SE3d poseWorldToCamera;
            poseWorldToCamera.setQuaternion(pose.first.getRotationQuatd());
            poseWorldToCamera.translation() = pose.first.getTranslation();
            poseWorldToCamera = poseWorldToCamera.inverse();

            const auto &translation = poseWorldToCamera.translation();
            const auto &rotationQuat = poseWorldToCamera.unit_quaternion();
            const auto &cameraIntr = pose.second;

            poseWorldToCameraTxTyTzByPoseNumber.push_back({translation[0], translation[1], translation[2]});
            orientationWorldToCameraqxyzwByPoseNumber.push_back(
                    {rotationQuat.x(), rotationQuat.y(), rotationQuat.z(), rotationQuat.w()});

            cameraModelByPoseNumber.emplace_back(pose.second);
            assert(poseWorldToCameraTxTyTzByPoseNumber[poseWorldToCameraTxTyTzByPoseNumber.size() - 1].size() ==
                   dimPose);
        }

        assert(pointsXYZbyIndex.size() == points.size());
        assert(keyPointInfo.size() == keyPointInfoByPoseNumberAndPointNumber.size());
        assert(posesCameraToWorld.size() == poseWorldToCameraTxTyTzByPoseNumber.size());
    }

    int BundleDepthAdjuster::getMaxNumberIterations() const {
        return iterations;
    }

    int BundleDepthAdjuster::getMaxNumberThreads() const {
        return maxNumberTreadsCeres;
    }

    void BundleDepthAdjuster::setMaxNumberIterations(int iterationsNumber) {
        assert(iterationsNumber >= 0);
        iterations = iterationsNumber;
    }

    void BundleDepthAdjuster::setMaxNumberThreads(int maxNumberThreadsToUse) {
        maxNumberTreadsCeres = maxNumberThreadsToUse;
    }

    ceres::CostFunction *
    BundleDepthAdjuster::DepthOnlyResidual::Create(double newObservedDepth,
                                                   const CameraRGBD &camera,
                                                   double estNormalizedDepth,
                                                   double devDividerDepth,
                                                   double medianResDepth) {

        return (new ceres::AutoDiffCostFunction<DepthOnlyResidual, 1, dimPoint, dimPose, dimOrientation>(
                new DepthOnlyResidual(newObservedDepth,
                                      camera,
                                      estNormalizedDepth,
                                      devDividerDepth,
                                      medianResDepth)));
    }

    ceres::CostFunction *BundleDepthAdjuster::ReprojectionOnlyResidual::Create(double newObservedX,
                                                                               double newObservedY,
                                                                               double scale,
                                                                               const CameraRGBD &camera,
                                                                               double estNormalizedReproj,
                                                                               double devDividerReproj,
                                                                               double medianResReproj) {

        return (new ceres::AutoDiffCostFunction<ReprojectionOnlyResidual, 2, dimPoint, dimPose, dimOrientation>(
                new ReprojectionOnlyResidual(newObservedX, newObservedY,
                                             scale, camera,
                                             estNormalizedReproj,
                                             devDividerReproj,
                                             medianResReproj)));
    }

    BundleDepthAdjuster::ReprojectionOnlyResidual::ReprojectionOnlyResidual(double newObservedX,
                                                                            double newObservedY,
                                                                            double scale,
                                                                            const CameraRGBD &cameraRgbd,
                                                                            double devNormalizedEstReproj,
                                                                            double devDividerReproj,
                                                                            double medianResReproj)
            : observedX(newObservedX),
              observedY(newObservedY),
              scaleKeyPoint(scale),
              camera(cameraRgbd),
              deviationEstimationNormalizedReproj(devNormalizedEstReproj),
              deviationDividerReproj(devDividerReproj),
              medianResidualReproj(medianResReproj) {}

    BundleDepthAdjuster::DepthOnlyResidual::DepthOnlyResidual(double newObservedDepth,
                                                              const CameraRGBD &cameraRgbd,
                                                              double devNormalizedEstDepth,
                                                              double devDividerDepth,
                                                              double medianResDepth)
            : observedDepth(newObservedDepth),
              camera(cameraRgbd),
              deviationEstimationNormalizedDepth(devNormalizedEstDepth),
              deviationDividerDepth(devDividerDepth),
              medianResidualDepth(medianResDepth) {}
}
