//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <chrono>

#include "computationHandlers/RelativePosesComputationHandler.h"

#include "poseGraph/PosesForEvaluation.h"
#include "reconstructor/TesterReconstruction.h"

TEST(testBAOptimized, visualizationDesk98) {

    gdr::ParamsRANSAC paramsRansacDefault;
    paramsRansacDefault.setInlierCoeff(0.5);
    paramsRansacDefault.setProjectionUsage(false);

    gdr::CameraRGBD structureIoCamera(583, 320, 583, 240);
    structureIoCamera.setDepthPixelDivider(1000.0);

    gdr::CameraRGBD kinectCamera(517.3, 318.6, 516.5, 255.3);
    kinectCamera.setDepthPixelDivider(5000.0);

    std::string assocFile = "assoc.txt";

    double rotErrorThreshold = 0.04;
    double translationErrorThreshold = 0.04;

    double minCoefficientOfBiggestComponent = 0.5;

    auto results = test::TesterReconstruction::testReconstruction("../../data/plant_sampled_19_3",
                                                                  kinectCamera,
                                                                  paramsRansacDefault,
                                                                  assocFile);

    double meanErrorL2BA = results.errorBA.translationError.MEAN;
    double meanErrorRotBA = results.errorBA.rotationError.MEAN;

    double meanErrorL2IRLS = results.errorIRLS.translationError.MEAN;
    double meanErrorRotIRLS = results.errorIRLS.rotationError.MEAN;

    ASSERT_GE(results.errorBA.numberOfPosesTrajectory,
              results.numberOfPosesInDataset * minCoefficientOfBiggestComponent);

    ASSERT_LE(meanErrorL2BA, translationErrorThreshold);
    ASSERT_LE(meanErrorL2IRLS, translationErrorThreshold);
    ASSERT_LE(meanErrorRotBA, rotErrorThreshold);
    ASSERT_LE(meanErrorRotIRLS, rotErrorThreshold);

}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}



