//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <mutex>
#include "boost/filesystem.hpp"

#include <tbb/parallel_for.h>
#include <tbb/concurrent_vector.h>

#include "readerDataset/readerTUM/ReaderTum.h"
#include <directoryTraversing/DirectoryReader.h>

#include "keyPoints/KeyPointsDepthDescriptor.h"
#include "keyPointDetectionAndMatching/FeatureDetectorMatcherCreator.h"
#include "relativePoseEstimators/EstimatorRelativePoseRobustCreator.h"
#include "relativePoseRefinement/RefinerRelativePoseCreator.h"

#include "computationHandlers/RelativePosesComputationHandler.h"

namespace gdr {

    namespace fs = boost::filesystem;

    RelativePosesComputationHandler::RelativePosesComputationHandler(const std::string &pathToImageDirectoryRGB,
                                                                     const std::string &pathToImageDirectoryD,
                                                                     const std::string &rgbToDassociationFile,
                                                                     const ParamsRANSAC &paramsRansacToSet,
                                                                     const CameraRGBD &cameraDefaultToSet) :
            paramsRansac(paramsRansacToSet),
            cameraDefault(cameraDefaultToSet) {

        auto rgbImagesAll = DirectoryReader::readPathsToImagesFromDirectorySorted(pathToImageDirectoryRGB);
        auto depthImagesAll = DirectoryReader::readPathsToImagesFromDirectorySorted(pathToImageDirectoryD);

        assert(!rgbImagesAll.empty());
        assert(!depthImagesAll.empty());

        std::vector<std::pair<double, double>> timeStampsRgbDepth;

        if (rgbToDassociationFile.empty()) {
            assert(rgbImagesAll.size() == depthImagesAll.size()
                   && "without assoc.txt file number of RGB and D frames should be equal");

            for (int timeStep = 0; timeStep < rgbImagesAll.size(); ++timeStep) {
                timeStampsRgbDepth.emplace_back(
                        std::make_pair(static_cast<double>(timeStep), static_cast<double>(timeStep)));
            }

        } else {

            AssociatedImages associatedImages = ReaderTUM::readAssocShortFilenameRgbToD(rgbToDassociationFile);

            const auto &rgbSet = associatedImages.timeAndPairedDepthByRgb;
            const auto &depthSet = associatedImages.timeAndPairedRgbByDepth;

            std::cout << "sets are rgb, d: " << rgbSet.size() << ' ' << depthSet.size() << std::endl;
            assert(!rgbSet.empty());
            assert(depthSet.size() == depthSet.size());

            auto iteratorByRgb = rgbSet.begin();
            auto iteratorByDepth = depthSet.begin();

            while (iteratorByRgb != rgbSet.end()
                   && iteratorByDepth != depthSet.end()) {
                const std::string &rgbShortName = iteratorByRgb->first;
                const std::string &depthShortName = iteratorByDepth->first;

                assert(rgbShortName == iteratorByDepth->second.second);
                assert(depthShortName == iteratorByRgb->second.second);

                double timeRgb = iteratorByRgb->second.first;
                double timeD = iteratorByDepth->second.first;
                timeStampsRgbDepth.emplace_back(std::make_pair(timeRgb, timeD));

                ++iteratorByDepth;
                ++iteratorByRgb;
            }

            std::vector<std::string> rgbImagesAssociated;
            std::vector<std::string> depthImagesAssociated;

            for (const auto &rgbFrame: rgbImagesAll) {
                fs::path rgbPath(rgbFrame);
                std::string nameToFind = rgbPath.filename().string();

                if (rgbSet.find(nameToFind) != rgbSet.end()) {
                    rgbImagesAssociated.emplace_back(rgbPath.string());
                }
            }

            for (const auto &depthFrame: depthImagesAll) {
                fs::path depthPath(depthFrame);

                if (depthSet.find(depthPath.filename().string()) != depthSet.end()) {
                    depthImagesAssociated.emplace_back(depthPath.string());
                }
            }

            std::cout << "sizes timestamps, rgb, depth "
                      << timeStampsRgbDepth.size() << ' '
                      << rgbImagesAssociated.size() << ' '
                      << depthImagesAssociated.size() << std::endl;
            assert(timeStampsRgbDepth.size() == rgbImagesAssociated.size());
            assert(rgbImagesAssociated.size() == depthImagesAssociated.size());
            assert(!rgbImagesAssociated.empty());

            rgbImagesAll = rgbImagesAssociated;
            depthImagesAll = depthImagesAssociated;
        }

        timestampsRgbDepthAssociated = timeStampsRgbDepth;
        assert(!rgbImagesAll.empty());
        assert(rgbImagesAll.size() == depthImagesAll.size());
        assert(rgbImagesAll.size() == timestampsRgbDepthAssociated.size());

        correspondenceGraph = std::make_unique<CorrespondenceGraph>(rgbImagesAll,
                                                                    depthImagesAll,
                                                                    cameraDefault);

        siftModule = FeatureDetectorMatcherCreator::getFeatureDetector(
                FeatureDetectorMatcherCreator::SiftDetectorMatcher::SIFTGPU);
        relativePoseEstimatorRobust = EstimatorRelativePoseRobustCreator::getEstimator(
                inlierCounter,
                paramsRansac,
                EstimatorRelativePoseRobustCreator::EstimatorMinimal::UMEYAMA,
                EstimatorRelativePoseRobustCreator::EstimatorScalable::UMEYAMA);
        relativePoseRefiner = RefinerRelativePoseCreator::getRefiner(RefinerRelativePoseCreator::RefinerType::ICPCUDA);

        //TODO: use multiple threads safely, not one
        threadPool = std::make_unique<ThreadPool>(1);
    }

    const CorrespondenceGraph &RelativePosesComputationHandler::getCorrespondenceGraph() const {
        return *correspondenceGraph;
    }

    void RelativePosesComputationHandler::setNumberOfThreadsCPU(int numberOfThreadsCPUToSet) {
        numberOfThreadsCPU = numberOfThreadsCPUToSet;
    }

    std::vector<std::vector<RelativeSE3>> RelativePosesComputationHandler::computeRelativePoses() {

        if (getPrintInformationCout()) {
            std::cout << "start computing descriptors" << std::endl;
        }

        std::vector<std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>>>
                keysDescriptorsAll = siftModule->getKeypoints2DDescriptorsAllImages(
                correspondenceGraph->getPathsRGB(),
                {0});

        const auto &imagesRgb = correspondenceGraph->getPathsRGB();
        const auto &imagesD = correspondenceGraph->getPathsD();

        for (int currentImage = 0; currentImage < keysDescriptorsAll.size(); ++currentImage) {

            keyPointsDepthDescriptor keyPointsDepthDescriptor = keyPointsDepthDescriptor::filterKeypointsByKnownDepth(
                    keysDescriptorsAll[currentImage],
                    imagesD[currentImage],
                    cameraDefault.getDepthPixelDivider());

            double timeRgb = timestampsRgbDepthAssociated[currentImage].first;
            double timeD = timestampsRgbDepthAssociated[currentImage].second;

            assert(std::abs(timeRgb - timeD) < 0.02);

            VertexCG currentVertex(currentImage,
                                   correspondenceGraph->getCameraDefault(),
                                   keyPointsDepthDescriptor,
                                   imagesRgb[currentImage],
                                   imagesD[currentImage],
                                   timeD);

            correspondenceGraph->addVertex(currentVertex);
        }

        std::vector<KeyPointsDescriptors> keyPointsDescriptorsToBeMatched;
        keyPointsDescriptorsToBeMatched.reserve(correspondenceGraph->getNumberOfPoses());

        const auto &vertices = correspondenceGraph->getVertices();
        assert(vertices.size() == correspondenceGraph->getNumberOfPoses());
        for (const auto &vertex: vertices) {
            keyPointsDescriptorsToBeMatched.emplace_back(KeyPointsDescriptors(
                    vertex.getKeyPoints(),
                    vertex.getDescriptors(),
                    vertex.getDepths()
            ));
        }

        assert(keyPointsDescriptorsToBeMatched.size() == vertices.size());
        correspondenceGraph->setPointMatchesRGB(siftModule->findCorrespondences(keyPointsDescriptorsToBeMatched));

        correspondenceGraph->decreaseDensity();
        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> allInlierKeyPointMatches;
        auto relativePoses = findTransformationRtMatrices(allInlierKeyPointMatches);
        assert(!allInlierKeyPointMatches.empty());
        correspondenceGraph->setInlierPointMatches(allInlierKeyPointMatches);

        correspondenceGraph->setRelativePoses(relativePoses);
        std::string poseFile = getPathRelativePose();
        correspondenceGraph->printRelativePosesFile(poseFile);

        return relativePoses;
    }

    void RelativePosesComputationHandler::setPathRelativePoseFile(const std::string &relativePoseFilePath) {
        relativePoseFileG2o = relativePoseFilePath;
    }

    const std::string &RelativePosesComputationHandler::getPathRelativePose() const {
        return relativePoseFileG2o;
    }

    std::vector<std::vector<RelativeSE3>> RelativePosesComputationHandler::findTransformationRtMatrices(
            std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &allInlierKeyPointMatches) const {

        int numberOfVertices = getNumberOfVertices();
        tbb::concurrent_vector<tbb::concurrent_vector<RelativeSE3>> transformationMatricesConcurrent(numberOfVertices);

        tbb::concurrent_vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> allInlierKeyPointMatchesTBB;

        const auto &vertices = correspondenceGraph->getVertices();
        const auto &keyPointMatches = correspondenceGraph->getKeyPointMatches();

        assert(keyPointMatches.size() == numberOfVertices);
        assert(numberOfVertices == vertices.size());

        tbb::parallel_for(0, static_cast<int>(numberOfVertices),
                          [this, &allInlierKeyPointMatchesTBB,
                                  &keyPointMatches, &vertices,
                                  &transformationMatricesConcurrent](int vertexNumberFrom) {

                              int i = vertexNumberFrom;
                              tbb::parallel_for(0, static_cast<int>(keyPointMatches[i].size()),
                                                [i, &transformationMatricesConcurrent, this,
                                                        &keyPointMatches, &vertices,
                                                        &allInlierKeyPointMatchesTBB](int vertexNumberTo) {

                                                    int j = vertexNumberTo;

                                                    const auto &match = keyPointMatches[i][j];
                                                    const auto &frameFromDestination = vertices[i];
                                                    const auto &frameToToBeTransformed = vertices[match.getFrameNumber()];

                                                    assert(frameToToBeTransformed.getIndex() >
                                                           frameFromDestination.getIndex());
                                                    bool success = true;
                                                    std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierKeyPointMatches;
                                                    auto cameraMotion = getTransformationRtMatrixTwoImages(i,
                                                                                                           j,
                                                                                                           inlierKeyPointMatches,
                                                                                                           success);

                                                    if (success) {

                                                        for (const auto &matchPair: inlierKeyPointMatches) {
                                                            allInlierKeyPointMatchesTBB.emplace_back(matchPair);
                                                        }

                                                        // fill info about relative pairwise transformations Rt
                                                        transformationMatricesConcurrent[i].push_back(
                                                                RelativeSE3(
                                                                        frameFromDestination.getIndex(),
                                                                        frameToToBeTransformed.getIndex(),
                                                                        cameraMotion
                                                                ));
                                                        transformationMatricesConcurrent[frameToToBeTransformed.getIndex()].push_back(
                                                                RelativeSE3(
                                                                        frameToToBeTransformed.getIndex(),
                                                                        frameFromDestination.getIndex(),
                                                                        cameraMotion.inverse()));
                                                    } else {}
                                                });
                          });

        std::vector<std::vector<RelativeSE3>> pairwiseTransformations(numberOfVertices);

        for (int i = 0; i < transformationMatricesConcurrent.size(); ++i) {
            for (const auto &transformation: transformationMatricesConcurrent[i]) {
                pairwiseTransformations[i].emplace_back(transformation);
            }
        }

        allInlierKeyPointMatches.clear();
        allInlierKeyPointMatches.reserve(allInlierKeyPointMatchesTBB.size());
        for (const auto &matchPair: allInlierKeyPointMatchesTBB) {
            allInlierKeyPointMatches.emplace_back(matchPair);
        }
        assert(allInlierKeyPointMatchesTBB.size() == allInlierKeyPointMatches.size());

        return pairwiseTransformations;
    }

    int RelativePosesComputationHandler::getNumberOfVertices() const {
        return correspondenceGraph->getNumberOfPoses();
    }

    SE3 RelativePosesComputationHandler::getTransformationRtMatrixTwoImages(
            int vertexFromDestDestination,
            int vertexInListToBeTransformedCanBeComputed,
            std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &keyPointMatches,
            bool &success,
            bool showMatchesOnImages) const {

        SE3 cR_t_umeyama;
        success = true;

        double inlierCoeff = paramsRansac.getInlierCoeff();

        if (inlierCoeff >= 1.0) {
            inlierCoeff = 1.0;
        }
        if (inlierCoeff < 0) {
            success = false;
            return cR_t_umeyama;
        }

        const auto &match = correspondenceGraph->getMatch(vertexFromDestDestination,
                                                          vertexInListToBeTransformedCanBeComputed);
        int minSize = match.getSize();

        if (minSize < paramsRansac.getInlierNumber()) {
            success = false;
            return cR_t_umeyama;
        }


        std::vector<Point3d> toBeTransformedPointsVector;
        std::vector<Point3d> destinationPointsVector;
        int vertexToBeTransformed = match.getFrameNumber();
        const auto &vertices = correspondenceGraph->getVertices();

        for (int i = 0; i < minSize; ++i) {

            double x_destination, y_destination, z_destination;
            int localIndexDestination = match.getKeyPointIndexDestinationAndToBeTransformed(i).first;
            const auto &siftKeyPointDestination = vertices[vertexFromDestDestination].getKeyPoint(
                    localIndexDestination);
            x_destination = siftKeyPointDestination.getX();
            y_destination = siftKeyPointDestination.getY();
            z_destination = vertices[vertexFromDestDestination].getKeyPointDepth(localIndexDestination);
            destinationPointsVector.emplace_back(Point3d(x_destination, y_destination, z_destination));

            double x_toBeTransformed, y_toBeTransformed, z_toBeTransformed;
            int localIndexToBeTransformed = match.getKeyPointIndexDestinationAndToBeTransformed(i).second;

            const auto &siftKeyPointToBeTransformed = vertices[vertexToBeTransformed].getKeyPoint(
                    localIndexToBeTransformed);
            x_toBeTransformed = siftKeyPointToBeTransformed.getX();
            y_toBeTransformed = siftKeyPointToBeTransformed.getY();
            z_toBeTransformed = vertices[vertexToBeTransformed].getKeyPointDepth(localIndexToBeTransformed);
            toBeTransformedPointsVector.emplace_back(Point3d(x_toBeTransformed, y_toBeTransformed, z_toBeTransformed));

            std::vector<std::pair<int, int>> points = {{vertexFromDestDestination, localIndexDestination},
                                                       {vertexToBeTransformed,     localIndexToBeTransformed}};

        }
        assert(toBeTransformedPointsVector.size() == minSize);
        assert(destinationPointsVector.size() == minSize);


        Eigen::Matrix4Xd toBeTransformedPoints = vertices[vertexFromDestDestination].getCamera()
                .getPointCloudXYZ1BeforeProjection(toBeTransformedPointsVector);
        Eigen::Matrix4Xd destinationPoints = vertices[match.getFrameNumber()].getCamera().
                getPointCloudXYZ1BeforeProjection(destinationPointsVector);

        assert(toBeTransformedPoints.cols() == minSize);
        assert(destinationPoints.cols() == minSize);

        const auto &cameraToBeTransformed = vertices[vertexFromDestDestination].getCamera();
        const auto &cameraDest = vertices[match.getFrameNumber()].getCamera();

        std::vector<int> inliersAgain;
        SE3 relativePoseLoRANSAC = relativePoseEstimatorRobust->estimateRelativePose(toBeTransformedPoints,
                                                                                     destinationPoints,
                                                                                     cameraToBeTransformed,
                                                                                     cameraDest,
                                                                                     success,
                                                                                     inliersAgain);

        if (!success) {
            return cR_t_umeyama;
        }

        auto inlierMatchesCorrespondingKeypointsLoRansac = findInlierPointCorrespondences(vertexFromDestDestination,
                                                                                          vertexInListToBeTransformedCanBeComputed,
                                                                                          relativePoseLoRANSAC);
        assert(inliersAgain.size() == inlierMatchesCorrespondingKeypointsLoRansac.size());
        assert(inliersAgain.size() >= inlierCoeff * toBeTransformedPoints.cols());

        bool successRefine = true;
        SE3 refinedByICPRelativePose = relativePoseLoRANSAC;
        refineRelativePose(vertices[vertexToBeTransformed],
                           vertices[vertexFromDestDestination],
                           refinedByICPRelativePose,
                           successRefine);

        auto inlierMatchesCorrespondingKeypointsAfterRefinement =
                findInlierPointCorrespondences(vertexFromDestDestination,
                                               vertexInListToBeTransformedCanBeComputed,
                                               refinedByICPRelativePose);

        int ransacInliers = inliersAgain.size();
        int ICPinliers = inlierMatchesCorrespondingKeypointsAfterRefinement.size();

        if (getPrintInformationCout()) {
            std::cout << "              ransac got " << ransacInliers << "/" << toBeTransformedPoints.cols()
                      << " vs " << ICPinliers << std::endl;
        }

        if (ransacInliers > ICPinliers) {
            // ICP did not refine the relative pose -- return umeyama result
            cR_t_umeyama = relativePoseLoRANSAC;

        } else {
            cR_t_umeyama = refinedByICPRelativePose;
            if (getPrintInformationCout()) {
                std::cout << "REFINED________________________________________________" << std::endl;
            }
            std::swap(inlierMatchesCorrespondingKeypointsAfterRefinement, inlierMatchesCorrespondingKeypointsLoRansac);
        }


        std::vector<std::pair<int, int>> matchesForVisualization;
        keyPointMatches = inlierMatchesCorrespondingKeypointsLoRansac;
        const auto &poseFrom = vertices[vertexFromDestDestination];
        const auto &poseTo = vertices[vertexToBeTransformed];

        return cR_t_umeyama;
    }

    std::vector<std::pair<double, double>>
    RelativePosesComputationHandler::getReprojectionErrorsXY(const Eigen::Matrix4Xd &destinationPoints,
                                                             const Eigen::Matrix4Xd &transformedPoints,
                                                             const CameraRGBD &cameraIntrinsics) {
        Eigen::Matrix3d intrinsicsMatrix = cameraIntrinsics.getIntrinsicsMatrix3x3();

        std::vector<std::pair<double, double>> errorsReprojection;
        errorsReprojection.reserve(destinationPoints.cols());
        assert(destinationPoints.cols() == transformedPoints.cols());

        for (int i = 0; i < destinationPoints.cols(); ++i) {
            Eigen::Vector3d homCoordDestination = intrinsicsMatrix * destinationPoints.col(i).topLeftCorner<3, 1>();
            Eigen::Vector3d homCoordTransformed = intrinsicsMatrix * transformedPoints.col(i).topLeftCorner<3, 1>();

            for (int j = 0; j < 2; ++j) {
                homCoordDestination[j] /= homCoordDestination[2];
                homCoordTransformed[j] /= homCoordTransformed[2];
            }
            double errorX = std::abs(homCoordTransformed[0] - homCoordDestination[0]);
            double errorY = std::abs(homCoordTransformed[1] - homCoordDestination[1]);
            errorsReprojection.emplace_back(std::make_pair(errorX, errorY));
        }

        return errorsReprojection;
    }

    std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
    RelativePosesComputationHandler::findInlierPointCorrespondences(int vertexFrom,
                                                                    int vertexInList,
                                                                    const SE3 &transformation) const {

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> correspondencesBetweenTwoImages;
        const auto &match = correspondenceGraph->getMatch(vertexFrom, vertexInList);
        const auto &vertices = correspondenceGraph->getVertices();
        int minSize = match.getSize();


        std::vector<Point3d> toBeTransformedPointsVector;
        std::vector<Point3d> destinationPointsVector;

        for (int i = 0; i < minSize; ++i) {

            double x_destination, y_destination, depth_destination;
            int localIndexDestination = match.getKeyPointIndexDestinationAndToBeTransformed(i).first;
            const auto &siftKeyPointDestination = vertices[vertexFrom].getKeyPoint(localIndexDestination);
            x_destination = siftKeyPointDestination.getX();
            y_destination = siftKeyPointDestination.getY();
            depth_destination = siftKeyPointDestination.getDepth();
            assert(std::abs(depth_destination - vertices[vertexFrom].getKeyPointDepth(localIndexDestination))
                   < 3 * std::numeric_limits<double>::epsilon());
            destinationPointsVector.emplace_back(Point3d(x_destination, y_destination, depth_destination));

            std::pair<std::pair<int, int>, KeyPointInfo> infoDestinationKeyPoint = {{vertexFrom, localIndexDestination},
                                                                                    KeyPointInfo(
                                                                                            siftKeyPointDestination,
                                                                                            vertexFrom)};

            double x_toBeTransformed, y_toBeTransformed, depth_toBeTransformed;
            int localIndexToBeTransformed = match.getKeyPointIndexDestinationAndToBeTransformed(i).second;
            int vertexToBeTransformed = match.getFrameNumber();
            const auto &siftKeyPointToBeTransformed = vertices[vertexToBeTransformed].getKeyPoint(
                    localIndexToBeTransformed);
            x_toBeTransformed = siftKeyPointToBeTransformed.getX();
            y_toBeTransformed = siftKeyPointToBeTransformed.getY();
            depth_toBeTransformed = siftKeyPointToBeTransformed.getDepth();

            assert(std::abs(
                    depth_toBeTransformed - vertices[vertexToBeTransformed].getKeyPointDepth(localIndexToBeTransformed))
                   < 3 * std::numeric_limits<double>::epsilon());

            toBeTransformedPointsVector.emplace_back(
                    Point3d(x_toBeTransformed, y_toBeTransformed, depth_toBeTransformed));


            std::pair<std::pair<int, int>, KeyPointInfo> infoToBeTransformedKeyPoint = {
                    {vertexToBeTransformed, localIndexToBeTransformed},
                    KeyPointInfo(siftKeyPointToBeTransformed, vertexToBeTransformed)};
            correspondencesBetweenTwoImages.push_back({infoDestinationKeyPoint, infoToBeTransformedKeyPoint});

        }
        assert(toBeTransformedPointsVector.size() == minSize);
        assert(destinationPointsVector.size() == minSize);

        const auto &poseToDestination = vertices[match.getFrameNumber()];

        Eigen::Matrix4Xd toBeTransformedPoints = vertices[vertexFrom].getCamera().
                getPointCloudXYZ1BeforeProjection(toBeTransformedPointsVector);
        Eigen::Matrix4Xd destinationPoints = poseToDestination.getCamera().getPointCloudXYZ1BeforeProjection(
                destinationPointsVector);

        Eigen::Matrix4Xd transformedPoints = transformation.getSE3().matrix() * toBeTransformedPoints;

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondences;
        std::vector<std::pair<double, int>> reprojectionInlierErrors = inlierCounter.calculateInlierProjectionErrors(
                toBeTransformedPoints,
                destinationPoints,
                vertices[vertexFrom].getCamera(),
                transformation,
                paramsRansac);

        assert(inlierCorrespondences.empty());
        assert(correspondencesBetweenTwoImages.size() == transformedPoints.cols());

        for (const auto &inlierErrorAndIndex: reprojectionInlierErrors) {
            int inlierNumber = inlierErrorAndIndex.second;
            assert(inlierNumber >= 0 && inlierNumber < correspondencesBetweenTwoImages.size());
            inlierCorrespondences.emplace_back(correspondencesBetweenTwoImages[inlierNumber]);
        }

        assert(reprojectionInlierErrors.size() == inlierCorrespondences.size());

        return inlierCorrespondences;
    }

    int RelativePosesComputationHandler::refineRelativePose(const VertexCG &vertexToBeTransformed,
                                                            const VertexCG &vertexDestination,
                                                            SE3 &initEstimationRelPos,
                                                            bool &refinementSuccess) const {

        MatchableInfo poseToBeTransformed(vertexToBeTransformed.getPathRGBImage(),
                                          vertexToBeTransformed.getPathDImage(),
                                          vertexToBeTransformed.getKeyPoints2D(),
                                          vertexToBeTransformed.getCamera());

        MatchableInfo poseDestination(vertexDestination.getPathRGBImage(),
                                      vertexDestination.getPathDImage(),
                                      vertexDestination.getKeyPoints2D(),
                                      vertexDestination.getCamera());
        refinementSuccess = relativePoseRefiner->refineRelativePose(poseToBeTransformed,
                                                                    poseDestination,
                                                                    initEstimationRelPos);
        assert(refinementSuccess);

        return 0;
    }

    std::vector<std::vector<int>>
    RelativePosesComputationHandler::bfsComputeConnectedComponents(std::vector<int> &componentNumberByPoseIndex) const {

        int totalNumberOfPoses = correspondenceGraph->getNumberOfPoses();
        std::vector<std::vector<int>> connectedComponents =
                GraphTraverser::bfsComputeConnectedComponents(*correspondenceGraph, componentNumberByPoseIndex);

        assert(componentNumberByPoseIndex.size() == totalNumberOfPoses);
        assert(totalNumberOfPoses == correspondenceGraph->getNumberOfPoses());

        int sumNumberOfPoses = 0;
        for (const auto &component: connectedComponents) {
            sumNumberOfPoses += component.size();
        }
        assert(sumNumberOfPoses == totalNumberOfPoses);

        return connectedComponents;
    }

    std::vector<std::unique_ptr<AbsolutePosesComputationHandler>>
    RelativePosesComputationHandler::splitGraphToConnectedComponents() const {
        auto components = GraphTraverser::splitGraphToConnectedComponents(*correspondenceGraph);
        std::vector<std::unique_ptr<AbsolutePosesComputationHandler>> connectedComponentsHandlers;
        connectedComponentsHandlers.reserve(components.size());

        for (auto &component: components) {
            connectedComponentsHandlers.emplace_back(std::make_unique<AbsolutePosesComputationHandler>(component));
        }

        return connectedComponentsHandlers;
    }

    void RelativePosesComputationHandler::bfsDrawToFile(const std::string &outFile) {
        GraphTraverser::bfsDrawToFile(*correspondenceGraph, outFile);
    }

    bool RelativePosesComputationHandler::getPrintInformationCout() const {
        return printInformationConsole;
    }

    void RelativePosesComputationHandler::setPrintInformationCout(bool printProgressToCout) {
        printInformationConsole = printProgressToCout;
    }
}
