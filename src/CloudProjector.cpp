//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "CloudProjector.h"
#include "pointCloud.h"
#include "BundleAduster.h"

#include "KeyPoint2D.h"

#include "boost/filesystem.hpp"

namespace gdr {

    void CloudProjector::setPoses(const std::vector<VertexCG *> &cameraPoses) {
        poses = cameraPoses;
        keyPointInfoByPose = std::vector<std::unordered_map<int, KeyPointInfo>>(cameraPoses.size());
    }

    int CloudProjector::addPoint(int pointIndex,
                                 const std::vector<KeyPointInfo> &poseNumbersAndProjectedKeyPointInfo) {

        assert(!poses.empty());
        maxPointIndex = std::max(pointIndex, maxPointIndex);

        for (const auto &poseAndInfo: poseNumbersAndProjectedKeyPointInfo) {
            int poseNumber = poseAndInfo.getObservingPoseNumber();
            assert(poseNumber >= 0);
            const KeyPointInfo &info = poseAndInfo;

            assert(poses.size() == keyPointInfoByPose.size());
            assert(poseNumber < keyPointInfoByPose.size() && poseNumber >= 0);
            keyPointInfoByPose[poseNumber].insert(std::make_pair(pointIndex, info));

        }

        return 0;
    }

    const Point3d &CloudProjector::getPointByIndex3d(int pointNumber3d) const {
        assert(pointNumber3d >= 0 && pointNumber3d < indexedPoints.size());
        return indexedPoints[pointNumber3d];
    }

    const VertexCG &CloudProjector::getPoseByPoseNumber(int poseNumber) const {
        assert(poseNumber >= 0 && poseNumber < poses.size());
        return *poses[poseNumber];
    }

    std::vector<std::pair<int, KeyPointInfo>> CloudProjector::getKeyPointsIndicesAndInfoByPose(int poseNumber) const {

        assert(poseNumber >= 0 && poseNumber < poses.size());

        assert(poseNumber < keyPointInfoByPose.size());

        std::vector<std::pair<int, KeyPointInfo>> resultKeyPointsObserved;

        for (const auto &pairIndexInfo: keyPointInfoByPose[poseNumber]) {
            resultKeyPointsObserved.push_back(pairIndexInfo);
        }

        return resultKeyPointsObserved;
    }

    int CloudProjector::getPoseNumber() const {
        return poses.size();
    }

    std::vector<Point3d> CloudProjector::setComputedPointsGlobalCoordinates() {

        assert(maxPointIndex >= 0);
        assert(indexedPoints.empty());
        indexedPoints = {};
        indexedPoints.reserve(maxPointIndex + 1);
        for (int i = 0; i < maxPointIndex + 1; ++i) {
            indexedPoints.push_back(Point3d(-1, -1, -1, i));
        }
        int pointsSize = indexedPoints.size();
        assert(pointsSize > 0);
        for (int i = 0; i < pointsSize; ++i) {
            assert(indexedPoints[i].getIndex() == i);
        }

        std::vector<std::vector<Eigen::Vector3d>> computedCoordinatesByPointIndex(pointsSize);

        numbersOfPosesObservingSpecificPoint = std::vector<std::vector<int>>(pointsSize);

        int posesSize = poses.size();
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

                assert(i == poses[i]->getIndex());
                assert(i == currentInfoBeforeProjection.getObservingPoseNumber());

                Eigen::Vector4d localCoordinatesBeforeProjection =
                        getPointBeforeProjection(infoBeforeProjection,
                                                 poses[i]->getCamera());


                // better with inverse CHANGE TODO
                Eigen::Vector4d globalCoordinates =
                        poses[i]->getEigenMatrixAbsolutePose4d() * localCoordinatesBeforeProjection;


//                std::cout << currentPointIndex << " / " << computedCoordinatesByPointIndex.size() << "__precomputed " << pointsSize << std::endl;
                assert(currentPointIndex < computedCoordinatesByPointIndex.size());
                assert(currentPointIndex >= 0);
                assert(computedCoordinatesByPointIndex.size() == pointsSize);

                Eigen::Vector3d coordinates3d = globalCoordinates.block<3, 1>(0, 0);
                for (int count = 0; count < 3; ++count) {
                    assert(std::abs(coordinates3d[count] - globalCoordinates[count]) < 1e-15);
                }
                computedCoordinatesByPointIndex[currentPointIndex].push_back(coordinates3d);

                const auto &camera = poses[i]->getCamera();
                auto cameraIntr = BundleAdjuster::getCameraIntr<double>(camera.fx, camera.cx, camera.fy, camera.cy);
                Eigen::Vector3d projected = cameraIntr * localCoordinatesBeforeProjection;
                double projectedX = projected[0] / projected[2];
                double projectedY = projected[1] / projected[2];
//                projectedX = 640 - projectedX;
//                projectedY = 480 - projectedY;

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
//        std::cout << "done listing points" << std::endl;

        for (int i = 0; i < computedCoordinatesByPointIndex.size(); ++i) {
//            std::cout << "step " << i << std::endl;
            Eigen::Vector3d sumCoordinates;
            sumCoordinates.setZero();

            assert(sumCoordinates.norm() < 3 * std::numeric_limits<double>::epsilon());
            assert(!computedCoordinatesByPointIndex[i].empty());
            for (const auto &coordinates: computedCoordinatesByPointIndex[i]) {
                sumCoordinates += coordinates;
            }

            assert(sumCoordinates.rows() == 3);
            for (int toDim = 0; toDim < sumCoordinates.rows(); ++toDim) {
//                std::cout << "dim index " << toDim << std::endl;
                sumCoordinates[toDim] /= computedCoordinatesByPointIndex[i].size();
            }

//            // only take point coordinates from first camera NO POINT COORDINATES AVERAGING
//            sumCoordinates = computedCoordinatesByPointIndex[i][0];



            indexedPoints[i].setEigenVector3dPointXYZ(sumCoordinates);
        }


        std::cout << "points are computed" << std::endl;
        for (int i = 0; i < pointsSize; ++i) {
            assert(indexedPoints[i].getIndex() == i);
        }
        return indexedPoints;
    }

    void CloudProjector::showPointsProjection(const std::vector<Point3d> &pointsGlobalCoordinates) const {

        bool showOnce = true;
        bool shown = false;
//        assert(shown);
        assert(numbersOfPosesObservingSpecificPoint.size() == pointsGlobalCoordinates.size());
        for (int i = 0; i < numbersOfPosesObservingSpecificPoint.size(); ++i) {
            assert(!numbersOfPosesObservingSpecificPoint[i].empty());

//            if (shown) {
//                break;
//            }
            // look at points far away from the (0;0;0)

//            if (numbersOfPosesObservingSpecificPoint[i][0] != 16 || numbersOfPosesObservingSpecificPoint[i].size() <= 1) //|| keyPointInfoByPose[numbersOfPosesObservingSpecificPoint[i][0]].find(i)->second.getDepth() < 2.0) {
//            {   continue;
//            }
            if (numbersOfPosesObservingSpecificPoint[i][0] < 10 ||
                numbersOfPosesObservingSpecificPoint[i].size() <= 3) {
                continue;
            }
            shown = true;


            std::vector<cv::Mat> imagesObservingCurrentPoint;

            for (const auto &poseIndex: numbersOfPosesObservingSpecificPoint[i]) {


                KeyPointInfo p = keyPointInfoByPose[poseIndex].find(i)->second;
                cv::KeyPoint keyPointToShow(cv::Point(p.getX(), p.getY()), 50);
                cv::Mat imageNoKeyPoints = cv::imread(poses.at(poseIndex)->getPathDImage(), cv::IMREAD_COLOR);


                const auto &camera = poses[poseIndex]->getCamera();
                auto cameraIntr = BundleAdjuster::getCameraIntr<double>(camera.fx, camera.cx, camera.fy, camera.cy);
                Eigen::Vector4d pointGlobal = pointsGlobalCoordinates[i].getEigenVector4dPointXYZ1();
                if (poseIndex == 0) {
                    std::cout << poses[poseIndex]->getEigenMatrixAbsolutePose4d() << std::endl;
                }



                // without inverse better.....
                // PositionMatrix * Global_ie_LocalZeroFrameCoordinates = LocalCurrentFrameCoordinates TODO
                Eigen::Vector3d projected =
                        cameraIntr * poses[poseIndex]->getEigenMatrixAbsolutePose4d().inverse() * pointGlobal;
                double projectedX = projected[0] / projected[2];
                double projectedY = projected[1] / projected[2];
//                projectedX = 640 - projectedX;
//                projectedY = 480 - projectedY;
                cv::KeyPoint keyPointComputed(cv::Point(projectedX, projectedY), 50);
                std::vector<cv::KeyPoint> keyPointsToShow = {keyPointToShow, keyPointComputed};

                cv::Mat imageKeyPoints = cv::imread(poses.at(poseIndex)->getPathDImage(), cv::IMREAD_COLOR);
                cv::drawKeypoints(imageNoKeyPoints, keyPointsToShow, imageKeyPoints);
                cv::putText(imageKeyPoints, //target image
                            "from sift(" + std::to_string((int) p.getX()) + ", " + std::to_string((int) p.getY())
                            + "), computed should be x,y: " + std::to_string((int) projectedX) + ", " +
                            std::to_string((int) projectedY), //text
                            cv::Point(p.getX(), p.getY()), //top-left position
                            cv::FONT_HERSHEY_DUPLEX,
                            0.4,
                            CV_RGB(118, 185, 0), //font color
                            1);
                cv::putText(imageKeyPoints, //target image
                            "computed point no " + std::to_string(i) + ". pose no " + std::to_string(poseIndex), //text
                            cv::Point(projectedX, projectedY), //top-left position
                            cv::FONT_HERSHEY_DUPLEX,
                            0.4,
                            CV_RGB(118, 185, 0), //font color
                            1);

                imagesObservingCurrentPoint.push_back(imageKeyPoints);
            }
            if (shown) {
                int cc = 0;
                for (const auto &image: imagesObservingCurrentPoint) {

                    cv::imshow("pose " + std::to_string(cc), image);
                    ++cc;
                }
                cv::waitKey(0);
                cv::destroyAllWindows();
            }
            if (showOnce) {
                break;
            }
//            assert(shown);

        }


    }


    std::vector<cv::Mat> CloudProjector::showPointsReprojectionError(
            const std::vector<Point3d> &pointsGlobalCoordinates,
            const std::string &pathToRGBDirectoryToSave,
            std::vector<double>& totalL2Errors,
            const CameraRGBD& camerasFromTo,
            int maxPointsToShow,
            bool drawCirclesKeyPoints,
            double quantil) const {

        double tresholdReprojInlier = 2.0;
        std::vector<cv::Mat> resultImages;
        assert(numbersOfPosesObservingSpecificPoint.size() == pointsGlobalCoordinates.size());

        std::vector<cv::Mat> imagesToShowKeyPoints;

        std::vector<double> sumL2Errors(poses.size(), 0);
        std::vector<std::vector<std::pair<KeyPoint2D, KeyPoint2D>>>
                keyPointsRealAndComputedByImageIndex(poses.size());

        for (int poseIndexComponent = 0; poseIndexComponent < poses.size(); ++poseIndexComponent) {
            cv::Mat imageNoKeyPoints = cv::imread(poses[poseIndexComponent]->getPathRGBImage(), cv::IMREAD_COLOR);
            imagesToShowKeyPoints.emplace_back(imageNoKeyPoints);
        }

        assert(imagesToShowKeyPoints.size() == poses.size());

        for (int i = 0; i < numbersOfPosesObservingSpecificPoint.size(); ++i) {

            assert(!numbersOfPosesObservingSpecificPoint[i].empty());

            for (const auto &poseIndex: numbersOfPosesObservingSpecificPoint[i]) {

                KeyPointInfo p = keyPointInfoByPose[poseIndex].find(i)->second;
                KeyPoint2D keyPointToShow(p.getX(), p.getY(), p.getScale(), p.getOrientation());
                keyPointToShow.setDepth(p.getDepth());

                const auto &camera = poses[poseIndex]->getCamera();
                auto cameraIntr = BundleAdjuster::getCameraIntr<double>(camera.fx, camera.cx, camera.fy, camera.cy);
                Eigen::Vector4d pointGlobal = pointsGlobalCoordinates[i].getEigenVector4dPointXYZ1();

                // without inverse better.....
                // PositionMatrix * Global_ie_LocalZeroFrameCoordinates = LocalCurrentFrameCoordinates TODO
                Eigen::Vector4d globalCoordinatesMoved = poses[poseIndex]->getEigenMatrixAbsolutePose4d().inverse() * pointGlobal;
                Eigen::Vector3d projected = cameraIntr * globalCoordinatesMoved;
                double projectedX = projected[0] / projected[2];
                double projectedY = projected[1] / projected[2];
                KeyPoint2D keyPointComputed(projectedX, projectedY, p.getScale(), p.getOrientation());
                keyPointComputed.setDepth(globalCoordinatesMoved[2]);
                assert(std::abs(projectedX - keyPointComputed.getX()) < std::numeric_limits<double>::epsilon());
//                double res = (keyPointComputed.pt. - keyPointToShow.pt);
                keyPointsRealAndComputedByImageIndex[poseIndex].emplace_back(
                        std::make_pair(keyPointToShow, keyPointComputed));

            }
        }

        for (int imageIndex = 0; imageIndex < imagesToShowKeyPoints.size(); ++imageIndex) {
            auto &pairsOfKeyPoints = keyPointsRealAndComputedByImageIndex[imageIndex];

            std::vector<double> errorsL2;
            for (const auto& lhs: pairsOfKeyPoints) {
                auto &leftKeyPointReal = lhs.first;
                auto &leftKeyPointComputed = lhs.second;
                Eigen::Vector3d computedCoordinates3DXYZ = poses[imageIndex]->getCamera()
                        .getCoordinates3D(leftKeyPointComputed.getX(),
                                          leftKeyPointComputed.getY(),
                                          leftKeyPointComputed.getDepth());
                assert(leftKeyPointComputed.isDepthUsable());


                Eigen::Vector3d observedCoordinates3DXYZ = poses[imageIndex]->getCamera()
                        .getCoordinates3D(leftKeyPointReal.getX(),
                                          leftKeyPointReal.getY(),
                                          leftKeyPointReal.getDepth());
                assert(leftKeyPointReal.isDepthUsable());
                double xError = leftKeyPointReal.getX() - leftKeyPointComputed.getX();
                double yError = leftKeyPointReal.getY() - leftKeyPointComputed.getY();
                double errorL2 = (observedCoordinates3DXYZ - computedCoordinates3DXYZ).norm();
//                double errorL2 = std::sqrt(std::pow(xError, 2) + std::pow(yError, 2));
                errorsL2.emplace_back(errorL2);
            }
            std::sort(errorsL2.begin(), errorsL2.end());
            sumL2Errors[imageIndex] = errorsL2[errorsL2.size() * quantil];

            std::sort(pairsOfKeyPoints.begin(), pairsOfKeyPoints.end(),
                      [](const std::pair<KeyPoint2D, KeyPoint2D> &lhs,
                         const std::pair<KeyPoint2D, KeyPoint2D> &rhs) {
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


            std::vector<KeyPoint2D> keyPointsToShow;
            std::vector<KeyPoint2D> keyPointsExactToShow;
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

            int linesToDrawDoubled = (maxPointsToShow < 0) ? (keyPointsToShow.size()) : (std::min(maxPointsToShow * 2, static_cast<int> (keyPointsToShow.size())));

            std::vector<cv::KeyPoint> keyPointsToShowCV;

            for (const auto& keyPoint: keyPointsToShow) {
                keyPointsToShowCV.emplace_back(cv::KeyPoint(cv::Point2f(keyPoint.getX(), keyPoint.getY()), keyPoint.getScale()));
            }
            if (drawCirclesKeyPoints) {
                cv::drawKeypoints(imageNoKeyPoints, keyPointsToShowCV, imageKeyPoints);
            } else {
                std::vector<cv::KeyPoint> keyPointsExactToShowCV;

                for (const auto& keyPoint: keyPointsExactToShow) {
                    keyPointsExactToShowCV.emplace_back(cv::KeyPoint(cv::Point2f(keyPoint.getX(), keyPoint.getY()), keyPoint.getScale()));
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
//            boost::filesystem::path pathToSaveImage = pathToRemove;
//            pathToSaveImage.append(shortNameImage);
//
            resultImages.emplace_back(imageKeyPoints);
//            cv::imwrite(pathToSaveImage.string(), imageKeyPoints);

//            cv::imshow(nameToSave, imageKeyPoints);
//            cv::waitKey(0);
        }

        totalL2Errors = sumL2Errors;
        return resultImages;

    }

    const std::vector<std::unordered_map<int, KeyPointInfo>>
    CloudProjector::getKeyPointInfoByPoseNumberAndPointClass() const {
        return keyPointInfoByPose;
    }


}
