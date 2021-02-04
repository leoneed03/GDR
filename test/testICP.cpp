//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "groundTruthTransformer.h"
#include "poseInfo.h"
#include "poseEstimation.h"
#include "CorrespondenceGraph.h"
#include "ICP.h"




TEST(testRelativePosesComputation, getPairwiseTransformations) {


    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
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


    ASSERT_LE(errorsStatsTR.first.meanError, 0.06);
    ASSERT_LE(errorsStatsTR.second.meanError, 0.06);
}


TEST(testRelativePosesComputation, getPairwiseTransformationsStandardIntrinsics) {


    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 525.0,
                                                 319.5, 525.0, 239.5);
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


    ASSERT_LE(errorsStatsTR.first.meanError, 0.07);
    ASSERT_LE(errorsStatsTR.second.meanError, 0.07);
}


TEST(testAbsolutePosesComputation, ICPOnly) {
    
    std::string absolutePosesICP = "../../data/files/absolutePosesICP_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesICP);
    std::vector<Eigen::Vector3d> absoluteTranslationsFromICP;

    std::string absolutePosesGroundTruth = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfoGroundTruth = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGroundTruth);
    std::vector<Eigen::Vector3d> absoluteTranslationsFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteTranslationsFromICP.push_back(posesInfo[i].getTranslation());
    }
    auto zeroT = absoluteTranslationsFromICP[0];

    for (auto& translations: absoluteTranslationsFromICP) {
        translations -= zeroT;
    }


    std::cout << "_______________________VS_______________________________________" << std::endl;
    for (int i = 0; i < posesInfoGroundTruth.size(); ++i) {
        absoluteTranslationsFromGroundTruth.push_back(posesInfoGroundTruth[i].getTranslation());
    }
    auto zeroTGroundTruth = absoluteTranslationsFromGroundTruth[0];

    for (auto& translations: absoluteTranslationsFromGroundTruth) {
        translations -= zeroTGroundTruth;
    }

    assert(absoluteTranslationsFromGroundTruth.size() == absoluteTranslationsFromGroundTruth.size());
    assert(absoluteTranslationsFromGroundTruth.size() > 0);
    std::cout << "______________________________________________________________" << std::endl;



    double sumErrors = 0;
    double sumRotError = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    assert(posesInfo.size() == posesInfoGroundTruth.size());

    for (int i = 0; i < posesInfo.size(); ++i) {

        if (i == 0) {
            Eigen::Matrix3d id;
            id.setIdentity();
            Eigen::Quaterniond quatID(id);
            assert(posesInfo[i].getOrientationQuat().angularDistance(quatID) < std::numeric_limits<double>::epsilon());
        }
        double currentL2Error = (absoluteTranslationsFromICP[i] - absoluteTranslationsFromGroundTruth[i]).norm();
        std::cout << i << ": L2 error translation \t" << currentL2Error << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

        auto zeroQuatGroundTruth = posesInfoGroundTruth[0].getOrientationQuat();
        auto zeroQuatICP = posesInfo[0].getOrientationQuat();

        auto currentQuatGroundTruth = posesInfoGroundTruth[i].getOrientationQuat();
        auto currentQuatICP = posesInfo[i].getOrientationQuat();
        double rotError = (zeroQuatGroundTruth.inverse().normalized() * currentQuatGroundTruth).angularDistance(zeroQuatICP.inverse().normalized() * currentQuatICP);
        std::cout << i << ": ang error Rotation \t" << rotError << std::endl;
        sumRotError += rotError;

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();
    double meanRotError = sumRotError / posesInfo.size();

    std::cout << "ICP for absolute translations result" << std::endl;
    std::cout << "E(error t) = " << meanError << std::endl;
    std::cout << "standard deviation(error t) = " << meanSquaredError - pow(meanError, 2) << std::endl;

    std::cout << "E(error R angular) = " << meanRotError << std::endl;
    ASSERT_LE(meanError, 0.15);
}




int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

