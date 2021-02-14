//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "poseEstimation.h"
#include "CorrespondenceGraph.h"
#include "poseInfo.h"
#include "groundTruthTransformer.h"

TEST(testRelativePosesComputation, PlantPASSED) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Sophus::SE3d> absolutePosesFromGroundTruth;
    Sophus::SE3d poseZeroGT;
    poseZeroGT.setQuaternion(posesInfo[0].getOrientationQuat().normalized());
    poseZeroGT.translation() = posesInfo[0].getTranslation();

    for (int i = 0; i < posesInfo.size(); ++i) {
        Sophus::SE3d absolutePoseGT;
        absolutePoseGT.setQuaternion(posesInfo[i].getOrientationQuat().normalized());
        absolutePoseGT.translation() = posesInfo[i].getTranslation();
        absolutePosesFromGroundTruth.push_back(poseZeroGT.inverse() * absolutePoseGT);
    }



    double errorRot = 0;
    double sumErrors = 0;
    double sumErrorsSquared = 0;
    int numberOfObservations = 0;

    for (int i = 0; i < correspondenceGraph.transformationRtMatrices.size(); ++i) {
        for (int j = 0; j < correspondenceGraph.transformationRtMatrices[i].size(); ++j) {
            auto& relativeTransformation = correspondenceGraph.transformationRtMatrices[i][j];

            int indexTo = relativeTransformation.getIndexTo();
            int indexFrom = relativeTransformation.getIndexFrom();

            if (indexFrom < indexTo) {
                ++numberOfObservations;
                Sophus::SE3d Rt = Sophus::SE3d::fitToSE3(relativeTransformation.innerTranformationRtMatrix);
                Sophus::SE3d relativeGT =
                        absolutePosesFromGroundTruth[indexFrom].inverse() * absolutePosesFromGroundTruth[indexTo];
//                relativeGT = relativeGT.inverse();
                double rotError = Rt.unit_quaternion().angularDistance(relativeGT.unit_quaternion());
                errorRot += rotError;
                double tError = (Rt.translation() - relativeGT.translation()).norm();
                sumErrors += tError;
                sumErrorsSquared += tError * tError;
            }
        }
    }

    double meanError = sumErrors / numberOfObservations;
    assert(absolutePosesFromGroundTruth.size() == 19);
    assert(absolutePosesFromGroundTruth.size() == correspondenceGraph.verticesOfCorrespondence.size());

    double meanErrorSquared = sumErrorsSquared / numberOfObservations;

    double meanErrorRot = errorRot / numberOfObservations;
    std::cout << "translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
//    std::cout << "standard deviation(error) = " << meanErrorSquared - pow(meanError, 2) << std::endl;
    std::cout << "______________________ROTATION REPORT______________________" << std::endl;
    std::cout << "Mean Rot angle error BA " << meanErrorRot << std::endl;

    ASSERT_LE(meanErrorRot, 0.02);
    ASSERT_LE(meanError, 0.02);
}


TEST(testRelativePosesComputation, getPairwiseTransformations) {

//517.3	516.5	318.6	255.3
    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
//    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
//                                                 525.0,
//                                                 319.5, 525.0, 239.5);
//    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
//    517.3,	516.5,	318.6,	255.3);
    correspondenceGraph.computeRelativePoses();
    std::string pathToGroundTruth = "../../data/plantDataset_19_3/groundtruth_new.txt";
    std::string estimatedPairWise = correspondenceGraph.relativePose;
    ASSERT_TRUE(correspondenceGraph.verticesOfCorrespondence.size() == 19);
    std::pair<gdr::errorStats, gdr::errorStats> errorsStatsTR = gdr::getErrorStatsTranslationRotationFromGroundTruthAndEstimatedPairWise(
            pathToGroundTruth, estimatedPairWise);
    std::cout << "=====================================" << std::endl;
    std::cout << "translation stats: " << errorsStatsTR.first.meanError << " with standard deviation "
              << errorsStatsTR.first.standartDeviation << std::endl;
    std::cout << "rotation    stats: " << errorsStatsTR.second.meanError << " with standard deviation "
              << errorsStatsTR.second.standartDeviation << std::endl;

    correspondenceGraph.printConnectionsRelative(std::cout, 15);

    ASSERT_LE(errorsStatsTR.first.meanError, 0.15);
    ASSERT_LE(errorsStatsTR.second.meanError, 0.25);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

