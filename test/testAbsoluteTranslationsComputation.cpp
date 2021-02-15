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
    double coefficient = 1.5;
    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();

    // compute absolute poses IRLS
    std::vector<Sophus::SE3d> posesIRLS;
    for (int i = 0; i < computedAbsoluteTranslationsIRLS.size(); ++i) {
        Sophus::SE3d poseIRLS;
        poseIRLS.setQuaternion(computedAbsoluteOrientationsRobust[i]);
        poseIRLS.translation() = computedAbsoluteTranslationsIRLS[i];
        posesIRLS.push_back(poseIRLS);
    }

    // set origin at zero pose [IRLS]
    Sophus::SE3d poseIRLSzero = posesIRLS[0];
    for (auto& poseIRLS: posesIRLS) {
        poseIRLS = poseIRLSzero.inverse() * poseIRLS;
    }

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Sophus::SE3d> posesGT;

    for (const auto& poseGT: posesInfo) {
        Sophus::SE3d poseSE3;
        poseSE3.setQuaternion(poseGT.getOrientationQuat());
        poseSE3.translation() = poseGT.getTranslation();
        posesGT.push_back(poseSE3);
    }
    Sophus::SE3d poseGTzero = posesGT[0];
    for (auto& poseGT: posesGT) {
        poseGT = poseGTzero.inverse() * poseGT;
    }

    double sumErrorT_IRLS = 0;
    double sumErrorR_IRLS = 0;

    double maxErrorR_IRLS = 0;
    double maxErrorT_IRLS = 0;

    assert(posesGT.size() == posesIRLS.size());
    assert(posesGT.size() == 19);

    for (int i = 0; i < posesGT.size(); ++i) {
        const auto& poseGT = posesGT[i];
        const auto& poseIRLS = posesIRLS[i];


        double errorR_IRLS = poseGT.unit_quaternion().angularDistance(poseIRLS.unit_quaternion());
        double errorT_IRLS = (poseGT.translation() - poseIRLS.translation()).norm();
        sumErrorR_IRLS += errorR_IRLS;
        sumErrorT_IRLS += errorT_IRLS;

        maxErrorR_IRLS = std::max(errorR_IRLS, maxErrorR_IRLS);
        maxErrorT_IRLS = std::max(errorT_IRLS, maxErrorT_IRLS);
    }

    double meanErrorT_IRLS_L2 = sumErrorT_IRLS / posesGT.size();
    double meanErrorR_IRLS_angDist = sumErrorR_IRLS / posesGT.size();

    std::cout << "__________IRLS test report 19 poses_____________" << std::endl;
    std::cout << "mean error translation: " << meanErrorT_IRLS_L2 << std::endl;
    std::cout << "mean error rotation: " << meanErrorR_IRLS_angDist << std::endl;
    std::cout << "MAX error translation: " << maxErrorT_IRLS << std::endl;
    std::cout << "MAX error rotation: " << maxErrorR_IRLS << std::endl;

    ASSERT_LE(meanErrorR_IRLS_angDist, 0.03);
    ASSERT_LE(meanErrorT_IRLS_L2, 0.02);

}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

