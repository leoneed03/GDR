//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <mutex>
#include "boost/filesystem.hpp"

#include <tbb/parallel_for.h>
#include <tbb/concurrent_vector.h>
#include <thread>

#include "readerDataset/readerTUM/ReaderTum.h"
#include <directoryTraversing/DirectoryReader.h>

#include "keyPoints/KeyPointsDepthDescriptor.h"
#include "keyPointDetectionAndMatching/FeatureDetectorMatcherCreator.h"
#include "relativePoseEstimators/EstimatorRelativePoseRobustCreator.h"
#include "relativePoseRefinement/RefinerRelativePoseCreator.h"

#include "computationHandlers/RelativePosesComputationHandler.h"

namespace gdr {

    namespace fs = boost::filesystem;

    RelativePosesComputationHandler::RelativePosesComputationHandler(const DatasetStructure &datasetStructure,
                                                                     const DatasetCameraDescriber &datasetDescriber,
                                                                     const ParamsRANSAC &paramsRansacToSet) :
            paramsRansac(paramsRansacToSet),
            cameraDefault(datasetDescriber.getDefaultCamera()) {

        const auto &rgbImagesAll = datasetStructure.pathsImagesRgb;
        const auto &depthImagesAll = datasetStructure.pathsImagesDepth;
        const auto &timeStampsRgbDepth = datasetStructure.timestampsRgbDepth;

        assert(rgbImagesAll.size() == depthImagesAll.size());
        assert(std::is_sorted(rgbImagesAll.begin(), rgbImagesAll.end()));
        assert(std::is_sorted(depthImagesAll.begin(), depthImagesAll.end()));

        for (const auto &rgbImagePathString: rgbImagesAll) {
            fs::path rgbImagePath(rgbImagePathString);
            bool found = true;

            camerasRgbByPoseIndex.emplace_back(datasetDescriber.findCameraRgb(rgbImagePath.filename().string(), found));
        }

        for (const auto &depthImagePathString: depthImagesAll) {
            fs::path depthImagePath(depthImagePathString);
            bool found = true;

            camerasDepthByPoseIndex.emplace_back(
                    datasetDescriber.findCameraDepth(depthImagePath.filename().string(), found));
        }

        timestampsRgbDepthAssociated = timeStampsRgbDepth;
        assert(!rgbImagesAll.empty());
        assert(rgbImagesAll.size() == depthImagesAll.size());
        assert(rgbImagesAll.size() == timestampsRgbDepthAssociated.size());
        assert(camerasRgbByPoseIndex.size() == camerasDepthByPoseIndex.size());
        assert(rgbImagesAll.size() == camerasRgbByPoseIndex.size());

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
    }

    const CorrespondenceGraph &RelativePosesComputationHandler::getCorrespondenceGraph() const {
        return *correspondenceGraph;
    }

    void RelativePosesComputationHandler::setNumberOfThreadsCPU(int numberOfThreadsCPUToSet) {
        numberOfThreadsCPU = numberOfThreadsCPUToSet;
    }

    std::vector<std::vector<RelativeSE3>> RelativePosesComputationHandler::computeRelativePoses(
            const std::vector<int> &gpuDeviceIndices) {

        assert(!gpuDeviceIndices.empty());
        deviceCudaICP = gpuDeviceIndices[0];

        if (getPrintInformationCout()) {
            std::cout << "start computing descriptors" << std::endl;
        }

        //sift detect
        std::vector<std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>>>
                keysDescriptorsAll = siftModule->getKeypoints2DDescriptorsAllImages(
                correspondenceGraph->getPathsRGB(),
                gpuDeviceIndices);

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

            assert(currentImage < camerasDepthByPoseIndex.size());

            VertexPose currentVertex(currentImage,
                                     camerasDepthByPoseIndex[currentImage],
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
        //Sift match
        correspondenceGraph->setPointMatchesRGB(siftModule->findCorrespondences(
                keyPointsDescriptorsToBeMatched,
                gpuDeviceIndices));

        correspondenceGraph->decreaseDensity();
        KeyPointMatches allInlierKeyPointMatches;
        auto relativePoses = findTransformationRtMatrices(allInlierKeyPointMatches);
        assert(!allInlierKeyPointMatches.getKeyPointMatchesVector().empty());
        correspondenceGraph->setInlierPointMatches(allInlierKeyPointMatches);

        correspondenceGraph->setRelativePoses(relativePoses);
        std::string poseFile = getPathRelativePose();
//        correspondenceGraph->printRelativePosesFile(poseFile);

        return relativePoses;
    }

    void RelativePosesComputationHandler::setPathRelativePoseFile(const std::string &relativePoseFilePath) {
        relativePoseFileG2o = relativePoseFilePath;
    }

    const std::string &RelativePosesComputationHandler::getPathRelativePose() const {
        return relativePoseFileG2o;
    }

    std::vector<std::vector<RelativeSE3>> RelativePosesComputationHandler::findTransformationRtMatrices(
            KeyPointMatches &allInlierKeyPointMatches) const {

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
                                                    KeyPointMatches inlierKeyPointMatches;
                                                    auto cameraMotion = getTransformationRtMatrixTwoImages(i,
                                                                                                           j,
                                                                                                           inlierKeyPointMatches,
                                                                                                           success);

                                                    if (success) {

                                                        for (auto &matchPair: inlierKeyPointMatches.getKeyPointMatchesVectorRef()) {
                                                            allInlierKeyPointMatchesTBB.emplace_back(
                                                                    std::move(matchPair));
                                                        }

                                                        // fill info about relative pairwise transformations Rt
                                                        transformationMatricesConcurrent[i].emplace_back(
                                                                std::move(RelativeSE3(
                                                                        frameFromDestination.getIndex(),
                                                                        frameToToBeTransformed.getIndex(),
                                                                        cameraMotion
                                                                )));
                                                        transformationMatricesConcurrent[frameToToBeTransformed.getIndex()].emplace_back(
                                                                std::move(RelativeSE3(
                                                                        frameToToBeTransformed.getIndex(),
                                                                        frameFromDestination.getIndex(),
                                                                        cameraMotion.inverse())));
                                                    }
                                                });
                          });

        std::vector<std::vector<RelativeSE3>> pairwiseTransformations(numberOfVertices);

        for (int i = 0; i < transformationMatricesConcurrent.size(); ++i) {
            pairwiseTransformations[i].reserve(transformationMatricesConcurrent.size());

            for (auto &transformation: transformationMatricesConcurrent[i]) {
                pairwiseTransformations[i].emplace_back(std::move(transformation));
            }
        }

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> allKeyPointsToSet;

        allKeyPointsToSet.reserve(allInlierKeyPointMatchesTBB.size());
        for (const auto &matchPair: allInlierKeyPointMatchesTBB) {
            allKeyPointsToSet.emplace_back(matchPair);
        }
        assert(allInlierKeyPointMatchesTBB.size() == allKeyPointsToSet.size());

        allInlierKeyPointMatches.setKeyPointMatches(allKeyPointsToSet);

        assert(allInlierKeyPointMatches.getKeyPointMatchesVector().size() == allInlierKeyPointMatchesTBB.size());

        return pairwiseTransformations;
    }

    int RelativePosesComputationHandler::getNumberOfVertices() const {
        return correspondenceGraph->getNumberOfPoses();
    }

    SE3 RelativePosesComputationHandler::getTransformationRtMatrixTwoImages(
            int vertexFromDestDestination,
            int vertexInListToBeTransformedCanBeComputed,
            KeyPointMatches &keyPointMatches,
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
        } else {
            assert(inliersAgain.size() >= paramsRansac.getInlierNumber());
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
                           KeyPointMatches(keyPointMatches),
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

    KeyPointMatches
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
            //TODO: use light structure and not vector here
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

        return KeyPointMatches(inlierCorrespondences);
    }

    int RelativePosesComputationHandler::refineRelativePose(const VertexPose &vertexToBeTransformed,
                                                            const VertexPose &vertexDestination,
                                                            const KeyPointMatches &keyPointMatches,
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
                                                                    KeyPointMatches(),
                                                                    initEstimationRelPos,
                                                                    deviceCudaICP);
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

    void RelativePosesComputationHandler::setDeviceCudaICP(int deviceCudaIcpToSet) {
        deviceCudaICP = deviceCudaIcpToSet;
    }
}
