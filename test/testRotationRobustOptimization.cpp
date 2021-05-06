//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>
#include <random>

#include "readerDataset/readerTUM/ReaderTum.h"
#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizerLogSO3.h"
#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"

#include "readerDataset/readerTUM/Evaluator.h"

void testRotationRobustOptimizationTemplate(const std::string &absolutePosesFile,
                                            double meanErrorThreshold = 0.02,
                                            double maxRotationErrorCoordinate = 0.05,
                                            int inlierRelativePosesPerVertex = 6,
                                            int outlierRelativePosesPerVertex = 1,
                                            int totalIterations = 20) {

    for (int iterations = 0; iterations < totalIterations; ++iterations) {

        std::vector<gdr::PoseFullInfo> absolutePosesInfo = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
                absolutePosesFile);
        std::vector<gdr::SE3> absolutePosesGroundTruth;

        int indexPoseFixed = iterations % static_cast<int>(absolutePosesInfo.size());

        for (const auto &poseGT: absolutePosesInfo) {
            absolutePosesGroundTruth.emplace_back(gdr::SE3(poseGT.getSophusPose()));
        }
        auto poseGTzero = absolutePosesGroundTruth[indexPoseFixed];

        for (auto &poseGT: absolutePosesGroundTruth) {
            poseGT = poseGTzero.inverse() * poseGT;
        }

        std::vector<gdr::RotationMeasurement> relativeRs;

        std::set<std::pair<int, int>> indexPairsFromAndTo;

        for (int indexFromInit = 0; indexFromInit < absolutePosesGroundTruth.size(); ++indexFromInit) {
            for (int indexToInit = indexFromInit + 1;
                 indexToInit < indexFromInit + inlierRelativePosesPerVertex;
                 ++indexToInit) {

                int indexTo = indexToInit % static_cast<int>(absolutePosesGroundTruth.size());
                int indexFrom = indexFromInit;

                if (indexTo < indexFrom) {
                    std::swap(indexTo, indexFrom);
                }

                if (indexPairsFromAndTo.find(std::make_pair(indexFrom, indexTo)) != indexPairsFromAndTo.end()) {
                    continue;
                } else {
                    indexPairsFromAndTo.insert(std::make_pair(indexFrom, indexTo));
                }

                ASSERT_LE(indexFrom, indexTo);
                ASSERT_LE(indexFrom, absolutePosesGroundTruth.size());
                ASSERT_LE(indexTo, absolutePosesGroundTruth.size());
                ASSERT_GE(indexFrom, 0);
                ASSERT_GE(indexTo, 0);

                gdr::RotationMeasurement relativeRotationInlier((absolutePosesGroundTruth[indexFrom].inverse()
                                                                 *
                                                                 absolutePosesGroundTruth[indexTo]).getRotationQuatd(),
                                                                indexFrom,
                                                                indexTo);

                relativeRs.emplace_back(std::move(relativeRotationInlier));
            }
        }

        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());

        for (int indexFromInit = 0; indexFromInit < absolutePosesGroundTruth.size() - 1; ++indexFromInit) {
            for (int outlierNumber = 0; outlierNumber < outlierRelativePosesPerVertex; ++outlierNumber) {

                std::uniform_int_distribution<> distrib(indexFromInit + 1, absolutePosesGroundTruth.size() - 1);

                int indexToBiggerRandom = distrib(randomNumberGenerator);

                ASSERT_GE(indexToBiggerRandom, indexFromInit);
                ASSERT_LE(indexToBiggerRandom, absolutePosesGroundTruth.size());

                if (indexPairsFromAndTo.find(std::make_pair(indexFromInit, indexToBiggerRandom)) !=
                    indexPairsFromAndTo.end()) {
                    continue;
                } else {
                    indexPairsFromAndTo.insert(std::make_pair(indexFromInit, indexToBiggerRandom));
                }

                auto quaternionOutlier = gdr::SO3::getRandomUnitQuaternion();

                gdr::RotationMeasurement relativeRotationOutlier(quaternionOutlier,
                                                                 indexFromInit,
                                                                 indexToBiggerRandom);

                relativeRs.emplace_back(std::move(relativeRotationOutlier));
            }
        }

        std::vector<gdr::SO3> orientationsAveraged =
                gdr::RotationAverager::shanonAveraging(relativeRs,
                                                       indexPoseFixed,
                                                       "test_RelativeRotationsForRobust.txt");

        ASSERT_EQ(orientationsAveraged.size(), absolutePosesGroundTruth.size());

        gdr::RotationRobustOptimizerLogSO3 optimizerRobust;
        std::vector<gdr::SO3> absoluteRotations =
                optimizerRobust.getOptimizedOrientation(orientationsAveraged, relativeRs, indexPoseFixed);

        ASSERT_EQ(absoluteRotations.size(), absolutePosesInfo.size());

        double sumError = 0;

        for (int j = 0; j < absoluteRotations.size(); ++j) {

            double error = absoluteRotations[j]
                    .getUnitQuaternion().angularDistance(
                    absolutePosesGroundTruth[j]
                            .getSO3().unit_quaternion());

            sumError += error;
        }

        double averageError = sumError / absoluteRotations.size();

        ASSERT_LE(averageError, meanErrorThreshold);
    }
}

TEST(testRotationAveraging, Poses19FromFileCorrespondencesPerVertexInliers3Outlier1) {

    testRotationRobustOptimizationTemplate("../../data/files/absolutePoses_19.txt");
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}