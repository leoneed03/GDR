//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include "poseEstimation.h"
#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"



TEST(testAbsoluteTranslationsComputation, IRLSForAbsoluteTranslations) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();



    for (int i = 0; i < computedAbsoluteOrientationsRobust.size(); ++i) {
        ASSERT_LE(computedAbsoluteOrientationsRobust[i].normalized().angularDistance(correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat().normalized()), 1e-10);
    }
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
    std::vector<Eigen::Vector3d> absoluteTranslationsFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteTranslationsFromGroundTruth.push_back(posesInfo[i].getTranslation());
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());

    double errorRot = 0;
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto quatShouldBeId = correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat();
        std::cout << "POSE " << i << std::endl;
        std::cout << quatShouldBeId.coeffs() << std::endl;

        std::cout << quatShouldBeId.x() << ' ' << quatShouldBeId.y() << ' ' << quatShouldBeId.z() << ' '
                  << quatShouldBeId.w() << std::endl;
        double dError = quatShouldBeId.angularDistance(posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << "error is " << dError << std::endl;
        errorRot += dError;
    }
    std::cout << "Mean Rot angle error " << errorRot / correspondenceGraph.verticesOfCorrespondence.size() << std::endl;
    auto zeroT = absoluteTranslationsFromGroundTruth[0];

    for (auto& translations: absoluteTranslationsFromGroundTruth) {
        translations -= zeroT;
    }


    std::cout << "_______________________VS_______________________________________" << std::endl;
    for (int i = 0; i < absoluteTranslationsFromGroundTruth.size(); ++i) {
        const auto& t = absoluteTranslationsFromGroundTruth[i];
        const auto& to = computedAbsoluteTranslationsIRLS[i];
        std::cout << i << ": \t" << t[0] << " \t" << t[1] << " \t" << t[2] << std::endl;
        std::cout << " : \t" << to[0] << " \t" << to[1] << " \t" << to[2] << std::endl;
    }

    std::cout << "______________________________________________________________" << std::endl;



    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

//    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/ba/absolutePoses_19_robust.txt";
    std::string outputName = "absolutePoses_19_robust.txt";
    std::ofstream computedPoses(outputName);

    assert(computedAbsoluteTranslationsIRLS.size() == absoluteTranslationsFromGroundTruth.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentL2Error = (absoluteTranslationsFromGroundTruth[i] - computedAbsoluteTranslationsIRLS[i]).norm();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        std::cout << i << ":\t" << currentL2Error << std::endl;
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        const auto& to = computedAbsoluteTranslationsIRLS[i];
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                  << quatComputed.w() << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();


    correspondenceGraph.printConnectionsRelative(std::cout);
    std::cout << "IRLS for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    ASSERT_LE(meanError, 0.15);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

