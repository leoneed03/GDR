//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>
#include <random>

#include "readerDataset/readerTUM/ReaderTum.h"
#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"

#include "readerDataset/readerTUM/Evaluator.h"

void testRotationAveragingTemplate(const std::string &absolutePosesFile,
                                   double meanErrorThreshold = 0.02,
                                   int totalIterations = 3) {

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

        for (int indexFrom = 0; indexFrom < absolutePosesGroundTruth.size() - 1; ++indexFrom) {
            for (int indexTo = indexFrom + 1; indexTo < absolutePosesGroundTruth.size(); ++indexTo) {

                if (indexFrom + 1 == indexTo || indexFrom + 2 == indexTo || indexFrom + 3 == indexTo) {
                    gdr::RotationMeasurement relativeRotation((absolutePosesGroundTruth[indexFrom].inverse()
                                                               * absolutePosesGroundTruth[indexTo]).getRotationQuatd(),
                                                              indexFrom,
                                                              indexTo);
                    relativeRs.emplace_back(std::move(relativeRotation));
                }
            }
        }

        std::vector<gdr::SO3> absoluteRotations =
                gdr::RotationAverager::shanonAveraging(relativeRs,
                                                       indexPoseFixed,
                                                       "test_RelativeRotations.txt");
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

TEST(testRotationAveraging, Poses19FromFileCorrespondencesPerVertex3AllInliers) {

    testRotationAveragingTemplate("../../data/files/absolutePoses_19.txt", 0.02, 10);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}