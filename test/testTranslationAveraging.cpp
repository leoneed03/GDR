//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>
#include <random>

#include "readerDataset/readerTUM/ReaderTum.h"
#include "absolutePoseEstimation/translationAveraging/TranslationAverager.h"

#include "readerDataset/readerTUM/Evaluator.h"

TEST(testTranslationAveraging, IRLS19PosesFromFileCorrespondencesPerVertex4SomeOutliers) {

    int successfulIterations = 0;
    int totalIterations = 40;

    double coefficientSuccessIterations = 0.6;

    int minSuccessIterations = static_cast<int>(coefficientSuccessIterations * totalIterations);
    double meanErrorTreshold = 1e-2;

    std::vector<double> errorsIRLS;
    std::vector<double> errorsPCG;

    for (int iterations = 0; iterations < totalIterations; ++iterations) {

        //read real SE3 groundtruth data
        std::string absolutePosesFile = "../../data/files/absolutePoses_19.txt";

        std::vector<gdr::PoseFullInfo> absolutePosesInfo = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
                absolutePosesFile);
        std::vector<gdr::SE3> absolutePosesGroundTruth;

        int indexPoseFixed = iterations % absolutePosesInfo.size();

        for (const auto &poseGT: absolutePosesInfo) {
            absolutePosesGroundTruth.emplace_back(gdr::SE3(poseGT.getSophusPose()));
        }
        auto poseGTzero = absolutePosesGroundTruth[indexPoseFixed];

        for (auto &poseGT: absolutePosesGroundTruth) {
            poseGT = poseGTzero.inverse() * poseGT;
        }

        std::vector<gdr::TranslationMeasurement> relativeTs;

        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());
        double maxTranslationCoord = 0.2;
        std::uniform_real_distribution<> distrib(-maxTranslationCoord, maxTranslationCoord);

        for (int indexFrom = 0; indexFrom < absolutePosesGroundTruth.size() - 1; ++indexFrom) {
            for (int indexTo = indexFrom + 1; indexTo < absolutePosesGroundTruth.size(); ++indexTo) {
                if (indexFrom + 1 == indexTo || indexFrom + 2 == indexTo || indexFrom + 3 == indexTo) {
                    gdr::TranslationMeasurement relPose((absolutePosesGroundTruth[indexFrom].inverse()
                                                         * absolutePosesGroundTruth[indexTo]).getTranslation(),
                                                        indexFrom,
                                                        indexTo);
                    relativeTs.push_back(relPose);
                }

                // add outlier measurements
                if (indexFrom + 4 == indexTo) {
                    Eigen::Vector3d outlierT;
                    outlierT[0] = distrib(randomNumberGenerator);
                    outlierT[1] = distrib(randomNumberGenerator);
                    outlierT[2] = distrib(randomNumberGenerator);
                    gdr::TranslationMeasurement relPose(outlierT, indexFrom, indexTo);
                    relativeTs.emplace_back(relPose);
                }
            }
        }

        bool successIRLS = true;
        std::vector<Eigen::Vector3d> absoluteTranslations =
                gdr::TranslationAverager::recoverTranslations(relativeTs,
                                                              absolutePosesGroundTruth,
                                                              indexPoseFixed).toVectorOfVectors();
        ASSERT_EQ(absoluteTranslations.size(), absolutePosesInfo.size());
        ASSERT_LE(absoluteTranslations[indexPoseFixed].norm(), std::numeric_limits<double>::epsilon());

        std::vector<Eigen::Vector3d> translationsPCG = absoluteTranslations;
        absoluteTranslations = gdr::TranslationAverager::recoverTranslationsIRLS(
                relativeTs,
                absolutePosesGroundTruth,
                absoluteTranslations,
                indexPoseFixed,
                successIRLS).toVectorOfVectors();

        ASSERT_LE(absoluteTranslations[indexPoseFixed].norm(), std::numeric_limits<double>::epsilon());

        double sumError = 0;
        double sumErrorPCG = 0;

        for (int j = 0; j < absoluteTranslations.size(); ++j) {
            double error = (absoluteTranslations[j] - absolutePosesGroundTruth[j].getTranslation()).norm();
            double errorPCG = (translationsPCG[j] - absolutePosesGroundTruth[j].getTranslation()).norm();
            sumError += error;
            sumErrorPCG += errorPCG;
        }

        double averageError = sumError / absoluteTranslations.size();
        double averageErrorPCG = sumErrorPCG / absoluteTranslations.size();

        if (averageError <= meanErrorTreshold && averageError < averageErrorPCG) {
            ++successfulIterations;
        }

        errorsPCG.emplace_back(averageErrorPCG);
        errorsIRLS.emplace_back(averageError);
    }

    assert(errorsPCG.size() == errorsIRLS.size());

    double medianErrorIRL = gdr::Evaluator::median(errorsIRLS);
    double medianErrorPCG = gdr::Evaluator::median(errorsPCG);

    std::cout << "===================================IRLS report=======================================" << std::endl;
    std::cout << "Successful IRLS solutions: " << successfulIterations << "/" << totalIterations << std::endl;
    std::cout << "Median mean ATE error: " << medianErrorIRL << std::endl;

    ASSERT_LE(medianErrorIRL, meanErrorTreshold);
    ASSERT_GE(successfulIterations, minSuccessIterations);
    ASSERT_LE(medianErrorIRL, medianErrorPCG);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}