//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "boost/filesystem.hpp"

#include "sparsePointCloud/CloudProjectorStl.h"
#include "keyPoints/KeyPoint2DAndDepth.h"


namespace gdr {

    int CloudProjectorStl::addPoint(int pointIndex,
                                    const std::vector<KeyPointInfo> &poseNumbersAndProjectedKeyPointInfo) {

        assert(getPoseNumber() > 0);
        maxPointIndex = std::max(pointIndex, maxPointIndex);

        for (const auto &poseAndInfo: poseNumbersAndProjectedKeyPointInfo) {
            int poseNumber = poseAndInfo.getObservingPoseNumber();
            assert(poseNumber >= 0);
            const KeyPointInfo &info = poseAndInfo;

            assert(getPoseNumber() == keyPointInfoByPose.size());
            assert(poseNumber < keyPointInfoByPose.size() && poseNumber >= 0);
            keyPointInfoByPose[poseNumber].insert(std::make_pair(pointIndex, info));

        }

        return 0;
    }

    const Point3d &CloudProjectorStl::getPointByIndex3d(int pointNumber3d) const {
        assert(pointNumber3d >= 0 && pointNumber3d < indexedPoints.size());
        return indexedPoints[pointNumber3d];
    }

    std::vector<std::pair<int, KeyPointInfo>> CloudProjectorStl::getKeyPointsIndicesAndInfoByPose(int poseNumber) const {

        assert(poseNumber >= 0 && poseNumber < getPoseNumber());

        assert(poseNumber < keyPointInfoByPose.size());

        std::vector<std::pair<int, KeyPointInfo>> resultKeyPointsObserved;

        for (const auto &pairIndexInfo: keyPointInfoByPose[poseNumber]) {
            resultKeyPointsObserved.push_back(pairIndexInfo);
        }

        return resultKeyPointsObserved;
    }

    int CloudProjectorStl::getPoseNumber() const {
        return poses.size();
    }

    std::vector<Point3d> CloudProjectorStl::computedPointsGlobalCoordinates() {

        assert(maxPointIndex >= 0);
        assert(indexedPoints.empty());
        indexedPoints = {};
        indexedPoints.reserve(maxPointIndex + 1);
        for (int i = 0; i < maxPointIndex + 1; ++i) {
            indexedPoints.emplace_back(Point3d(-1, -1, -1, i));
        }
        int pointsSize = indexedPoints.size();
        assert(pointsSize > 0);
        for (int i = 0; i < pointsSize; ++i) {
            assert(indexedPoints[i].getIndex() == i);
        }

        std::vector<std::vector<Eigen::Vector3d>> computedCoordinatesByPointIndex(pointsSize);

        numbersOfPosesObservingSpecificPoint = std::vector<std::vector<int>>(pointsSize);

        int posesSize = getPoseNumber();
        for (int i = 0; i < posesSize; ++i) {
            for (const auto &observedPoints: keyPointInfoByPose[i]) {

                const KeyPointInfo &currentInfoBeforeProjection = observedPoints.second;

                assert(currentInfoBeforeProjection.isInitialized());
                int currentPointIndex = observedPoints.first;

                assert(currentPointIndex >= 0 && currentPointIndex < numbersOfPosesObservingSpecificPoint.size());
                numbersOfPosesObservingSpecificPoint[currentPointIndex].push_back(i);

                double newX = currentInfoBeforeProjection.getX();
                double newY = currentInfoBeforeProjection.getY();
                double newZ = currentInfoBeforeProjection.getDepth();

                std::vector<double> infoBeforeProjection = {newX, newY, newZ, 1.0};

                assert(i == poses[i].getIndex());
                assert(i == currentInfoBeforeProjection.getObservingPoseNumber());

                Eigen::Vector4d localCoordinatesBeforeProjection =
                        poses[i].getCamera().getCoordinatesBeforeProjectionXYZ1(newX, newY, newZ);
                Eigen::Vector4d globalCoordinates =
                        poses[i].getPoseSE3().getSE3().matrix() * localCoordinatesBeforeProjection;
                assert(currentPointIndex < computedCoordinatesByPointIndex.size());
                assert(currentPointIndex >= 0);
                assert(computedCoordinatesByPointIndex.size() == pointsSize);

                Eigen::Vector3d coordinates3d = globalCoordinates.block<3, 1>(0, 0);
                for (int count = 0; count < 3; ++count) {
                    assert(std::abs(coordinates3d[count] - globalCoordinates[count]) < 1e-15);
                }
                computedCoordinatesByPointIndex[currentPointIndex].push_back(coordinates3d);

                const auto &camera = poses[i].getCamera();
                auto cameraIntr = camera.getIntrinsicsMatrix3x4();
                Eigen::Vector3d projected = cameraIntr * localCoordinatesBeforeProjection;
                double projectedX = projected[0] / projected[2];
                double projectedY = projected[1] / projected[2];

                bool okX = std::abs(projectedX - currentInfoBeforeProjection.getX()) < 1e-10;
                bool okY = std::abs(projectedY - currentInfoBeforeProjection.getY()) < 1e-10;

                if (!okX || !okY) {
                    std::cout << "error is (abs): " << std::abs(projectedX - currentInfoBeforeProjection.getX())
                              << std::endl;
                    std::cout << projectedX << " vs " << currentInfoBeforeProjection.getX() << std::endl;
                    std::cout << projectedY << " vs " << currentInfoBeforeProjection.getY() << std::endl;
                }
                assert(okX);
                assert(okY);
            }
        }


        // test visualize
        for (auto &posesObservingPoint: numbersOfPosesObservingSpecificPoint) {
            std::sort(posesObservingPoint.begin(), posesObservingPoint.end());
        }
        int pointIndexObseredByMaxPoses = -1;
        int maxPoses = 0;

        for (int i = 0; i < numbersOfPosesObservingSpecificPoint.size(); ++i) {
            if (numbersOfPosesObservingSpecificPoint[i].size() >= maxPoses) {
                maxPoses = numbersOfPosesObservingSpecificPoint[i].size();
                pointIndexObseredByMaxPoses = i;
            }
        }
        assert(pointIndexObseredByMaxPoses > -1);

        for (int i = 0; i < computedCoordinatesByPointIndex.size(); ++i) {
            Eigen::Vector3d sumCoordinates;
            sumCoordinates.setZero();

            assert(sumCoordinates.norm() < 3 * std::numeric_limits<double>::epsilon());
            assert(!computedCoordinatesByPointIndex[i].empty());
            for (const auto &coordinates: computedCoordinatesByPointIndex[i]) {
                sumCoordinates += coordinates;
            }

            assert(sumCoordinates.rows() == 3);
            for (int toDim = 0; toDim < sumCoordinates.rows(); ++toDim) {
                sumCoordinates[toDim] /= computedCoordinatesByPointIndex[i].size();
            }

            indexedPoints[i].setEigenVector3dPointXYZ(sumCoordinates);
        }

        for (int i = 0; i < pointsSize; ++i) {
            assert(indexedPoints[i].getIndex() == i);
        }
        return indexedPoints;
    }

    std::vector<cv::Mat> CloudProjectorStl::showPointsReprojectionError(
            const std::vector<Point3d> &pointsGlobalCoordinates,
            const std::string &pathToRGBDirectoryToSave,
            std::vector<double> &totalL2Errors,
            const CameraRGBD &camerasFromTo,
            int maxPointsToShow,
            bool drawCirclesKeyPoints,
            double quantil) const {

        double tresholdReprojInlier = 2.0;
        std::vector<cv::Mat> resultImages;
        assert(numbersOfPosesObservingSpecificPoint.size() == pointsGlobalCoordinates.size());

        std::vector<cv::Mat> imagesToShowKeyPoints;

        std::vector<double> sumL2Errors(getPoseNumber(), 0);
        std::vector<std::vector<std::pair<KeyPoint2DAndDepth, KeyPoint2DAndDepth>>>
                keyPointsRealAndComputedByImageIndex(getPoseNumber());

        for (int poseIndexComponent = 0; poseIndexComponent < getPoseNumber(); ++poseIndexComponent) {
            cv::Mat imageNoKeyPoints = cv::imread(poses[poseIndexComponent].getPathRGB(), cv::IMREAD_COLOR);
            imagesToShowKeyPoints.emplace_back(imageNoKeyPoints);
        }

        assert(imagesToShowKeyPoints.size() == getPoseNumber());

        for (int i = 0; i < numbersOfPosesObservingSpecificPoint.size(); ++i) {

            assert(!numbersOfPosesObservingSpecificPoint[i].empty());

            for (const auto &poseIndex: numbersOfPosesObservingSpecificPoint[i]) {

                KeyPointInfo p = keyPointInfoByPose[poseIndex].find(i)->second;
                KeyPoint2DAndDepth keyPointToShow(p.getX(), p.getY(), p.getScale(), p.getOrientation());
                keyPointToShow.setDepth(p.getDepth());

                const auto &camera = poses[poseIndex].getCamera();
                auto cameraIntr = camera.getIntrinsicsMatrix3x4();
                Eigen::Vector4d pointGlobal = pointsGlobalCoordinates[i].getEigenVector4dPointXYZ1();

                Eigen::Vector4d globalCoordinatesMoved =
                        poses[poseIndex].getPoseSE3().getSE3().inverse().matrix() * pointGlobal;
                Eigen::Vector3d projected = cameraIntr * globalCoordinatesMoved;
                double projectedX = projected[0] / projected[2];
                double projectedY = projected[1] / projected[2];
                KeyPoint2DAndDepth keyPointComputed(projectedX, projectedY, p.getScale(), p.getOrientation());
                keyPointComputed.setDepth(globalCoordinatesMoved[2]);
                assert(std::abs(projectedX - keyPointComputed.getX()) < std::numeric_limits<double>::epsilon());
                keyPointsRealAndComputedByImageIndex[poseIndex].emplace_back(
                        std::make_pair(keyPointToShow, keyPointComputed));

            }
        }

        for (int imageIndex = 0; imageIndex < imagesToShowKeyPoints.size(); ++imageIndex) {
            auto &pairsOfKeyPoints = keyPointsRealAndComputedByImageIndex[imageIndex];

            std::vector<double> errorsL2;
            for (const auto &lhs: pairsOfKeyPoints) {
                auto &leftKeyPointReal = lhs.first;
                auto &leftKeyPointComputed = lhs.second;
                Eigen::Vector3d computedCoordinates3DXYZ = poses[imageIndex].getCamera()
                        .getCoordinates3D(leftKeyPointComputed.getX(),
                                          leftKeyPointComputed.getY(),
                                          leftKeyPointComputed.getDepth());
                assert(leftKeyPointComputed.isDepthUsable());


                Eigen::Vector3d observedCoordinates3DXYZ = poses[imageIndex].getCamera()
                        .getCoordinates3D(leftKeyPointReal.getX(),
                                          leftKeyPointReal.getY(),
                                          leftKeyPointReal.getDepth());
                assert(leftKeyPointReal.isDepthUsable());
                double errorL2 = (observedCoordinates3DXYZ - computedCoordinates3DXYZ).norm();
                errorsL2.emplace_back(errorL2);
            }
            std::sort(errorsL2.begin(), errorsL2.end());
            sumL2Errors[imageIndex] = errorsL2[errorsL2.size() * quantil];

            std::sort(pairsOfKeyPoints.begin(), pairsOfKeyPoints.end(),
                      [](const std::pair<KeyPoint2DAndDepth, KeyPoint2DAndDepth> &lhs,
                         const std::pair<KeyPoint2DAndDepth, KeyPoint2DAndDepth> &rhs) {
                          auto &leftKeyPointReal = lhs.first;
                          auto &leftKeyPointComputed = lhs.second;

                          auto &rightKeyPointReal = rhs.first;
                          auto &rightKeyPointComputed = rhs.second;


                          return std::pow(leftKeyPointReal.getX() - leftKeyPointComputed.getX(), 2) +
                                 std::pow(leftKeyPointReal.getY() - leftKeyPointComputed.getY(), 2)
                                 >
                                 std::pow(rightKeyPointReal.getX() - rightKeyPointComputed.getX(), 2) +
                                 std::pow(rightKeyPointReal.getY() - rightKeyPointComputed.getY(), 2);
                      });

            if (maxPointsToShow >= 0 && maxPointsToShow < pairsOfKeyPoints.size()) {
                pairsOfKeyPoints.resize(maxPointsToShow);
            }
            auto &imageNoKeyPoints = imagesToShowKeyPoints[imageIndex];
            auto imageKeyPoints = imageNoKeyPoints;


            std::vector<KeyPoint2DAndDepth> keyPointsToShow;
            std::vector<KeyPoint2DAndDepth> keyPointsExactToShow;
            for (int keyPointPairIndex = 0;
                 keyPointPairIndex < keyPointsRealAndComputedByImageIndex[imageIndex].size(); ++keyPointPairIndex) {
                const auto &keyPointsPair = keyPointsRealAndComputedByImageIndex[imageIndex][keyPointPairIndex];
                const auto &keyPointReal = keyPointsPair.first;
                const auto &keyPointComputed = keyPointsPair.second;
                keyPointsToShow.emplace_back(keyPointsPair.first);
                keyPointsToShow.emplace_back(keyPointsPair.second);
                double errorReproj = std::sqrt(std::pow(keyPointReal.getX() - keyPointComputed.getX(), 2) +
                                               std::pow(keyPointReal.getY() - keyPointComputed.getY(), 2));
                if (errorReproj < tresholdReprojInlier) {
                    keyPointsExactToShow.emplace_back(keyPointsPair.first);
                    keyPointsExactToShow.emplace_back(keyPointsPair.second);
                }
            }

            int linesToDrawDoubled = (maxPointsToShow < 0) ? (static_cast<int> (keyPointsToShow.size())) :
                                     (std::min(maxPointsToShow * 2, static_cast<int> (keyPointsToShow.size())));

            std::vector<cv::KeyPoint> keyPointsToShowCV;

            for (const auto &keyPoint: keyPointsToShow) {
                keyPointsToShowCV.emplace_back(cv::KeyPoint(cv::Point2f(keyPoint.getX(), keyPoint.getY()),
                                                            keyPoint.getScale()));
            }
            if (drawCirclesKeyPoints) {
                cv::drawKeypoints(imageNoKeyPoints, keyPointsToShowCV, imageKeyPoints);
            } else {
                std::vector<cv::KeyPoint> keyPointsExactToShowCV;

                for (const auto &keyPoint: keyPointsExactToShow) {
                    keyPointsExactToShowCV.emplace_back(
                            cv::KeyPoint(cv::Point2f(keyPoint.getX(), keyPoint.getY()), keyPoint.getScale()));
                }
                cv::drawKeypoints(imageNoKeyPoints, keyPointsExactToShowCV, imageKeyPoints);
            }


            for (int i = 0; i < linesToDrawDoubled; i += 2) {
                int indexReal = i;
                int indexComuted = i + 1;
                cv::line(imageKeyPoints, keyPointsToShowCV[indexReal].pt, keyPointsToShowCV[indexComuted].pt,
                         cv::Scalar(0, 255, 0), 1, CV_AA);
            }

            std::string shortNameImage = std::to_string(imageIndex) + ".png";
            resultImages.emplace_back(imageKeyPoints);
        }

        totalL2Errors = sumL2Errors;
        return resultImages;

    }

    const std::vector<std::unordered_map<int, KeyPointInfo>> &
    CloudProjectorStl::getKeyPointInfoByPoseNumberAndPointClass() const {
        return keyPointInfoByPose;
    }

    void CloudProjectorStl::setPoses(const std::vector<SE3> &posesSE3Refined) {
        assert(poses.size() == posesSE3Refined.size());

        for (int i = 0; i < posesSE3Refined.size(); ++i) {
            poses[i].setPoseSE3(posesSE3Refined[i]);
        }
    }

    void CloudProjectorStl::setPoints(const std::vector<Point3d> &pointsRefined) {
        assert(pointsRefined.size() == indexedPoints.size());
        indexedPoints = pointsRefined;
    }

    void CloudProjectorStl::setCameraPoses(const std::vector<ProjectableInfo> &cameraPoses) {
        poses = cameraPoses;
        keyPointInfoByPose = std::vector<std::unordered_map<int, KeyPointInfo>>(cameraPoses.size());
    }

}
