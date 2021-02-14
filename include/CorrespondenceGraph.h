//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SIFTGPU_CG_H
#define GDR_SIFTGPU_CG_H

#include <queue>

#include "CloudProjector.h"
#include "VertexCG.h"
#include "transformationRt.h"
#include "cameraRGBD.h"
#include "siftModule.h"
#include "rotationAveraging.h"
#include "quaternions.h"
#include "errors.h"
#include "umeyama.h"
#include "Vectors3d.h"

namespace gdr {
    struct Match {
        int frameNumber;
        std::vector<std::pair<int, int>> matchNumbers;

        Match(int newFrameNumber, const std::vector<std::pair<int, int>> &newMatchNumbers) :
                frameNumber(newFrameNumber),
                matchNumbers(newMatchNumbers) {};
    };

    struct CorrespondenceGraph {

        PointMatcher pointMatcher;
        CloudProjector cloudProjector;
        CameraRGBD cameraRgbd;
        SiftModule siftModule;
        std::vector<VertexCG> verticesOfCorrespondence;
        int maxVertexDegree = 20;
        int numIterations = 100;
        std::vector<std::vector<Match>> matches;
        std::vector<std::vector<transformationRtMatrix>> transformationRtMatrices;
        double neighbourhoodRadius = 0.05;
        int minNumberOfMatches = 15;
        const std::string redCode = "\033[0;31m";
        const std::string resetCode = "\033[0m";
        const std::string relativePose = "relativeRotations.txt";
        const std::string absolutePose = "absoluteRotations.txt";
        std::vector<std::string> imagesRgb;
        std::vector<std::string> imagesD;
        std::string pathToImageDirectoryRGB;
        std::string pathToImageDirectoryD;
        int totalMeausedRelativePoses = 0;
        int refinedPoses = 0;

        const CloudProjector& getCloudProjector() const;

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondencesPoints;

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
        findInlierPointCorrespondences(int vertexFrom,
                                       int vertexInList,
                                       double inlierCoeff,
                                       Eigen::Matrix4d &transformation,
                                       bool isICP);

        CorrespondenceGraph(const std::string &pathToImageDirectoryRGB, const std::string &pathToImageDirectoryD,
                            float fx,
                            float cx, float fy, float cy);

        int refineRelativePose(const VertexCG &vertexToBeTransformed, const VertexCG &vertexDestination,
                               Eigen::Matrix4d &initEstimationRelPos, bool &success);

        int findCorrespondences();

        int findTransformationRtMatrices();

        void decreaseDensity();

        // each pair is poseNumber and point's local index
        // paired with its KeyPointInfo
        // pairs are grouped in one vector if representing same global point

        void computePointClasses(
                const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &matchesBetweenPoints);

        void computePointClasses();

        Eigen::Matrix4d
        getTransformationRtMatrixTwoImages(int vertexFromDestOrigin, int vertexInListToBeTransformedCanBeComputed,
                                           bool &success,
                                           bool useProjection = false,
                                           double inlierCoeff = 0.6,
                                           double maxProjectionErrorPixels = 1.0);

        void printConnectionsRelative(std::ostream &os, int space = 10);

        int printAbsolutePoses(std::ostream &os, int space = 10);

        std::vector<Eigen::Quaterniond> performRotationAveraging();


        std::vector<Eigen::Quaterniond> optimizeRotationsRobust();

        int printRelativePosesFile(const std::string &outPutFileRelativePoses);

        std::vector<int> bfs(int currentVertex);

        int computeRelativePoses();

        std::vector<Eigen::Matrix4d> getAbsolutePosesEigenMatrix4d() const;

        std::vector<Eigen::Vector3d> optimizeAbsoluteTranslations(int indexFixedToZero = 0);

        std::vector<Sophus::SE3d> performBundleAdjustment(int indexFixedToZero = 0);

        std::vector<Sophus::SE3d> performBundleAdjustmentUsingDepth(int indexFixedToZero = 0);
    };
}

#endif
