//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/AbsolutePosesComputationHandler.h"

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <chrono>

#include "readerDataset/readerTUM/ReaderTum.h"

#include "computationHandlers/RelativePosesComputationHandler.h"
#include "computationHandlers/ModelCreationHandler.h"

#include "readerDataset/readerTUM/Evaluator.h"

#include "poseGraph/PosesForEvaluation.h"
#include "reconstructor/TesterReconstruction.h"


TEST(testBAOptimized, visualizationDesk98) {

    gdr::ParamsRANSAC paramsRansacDefault;
    paramsRansacDefault.setProjectionUsage(false);

    gdr::CameraRGBD structureIoCamera(583, 320, 583, 240);
    structureIoCamera.setDepthPixelDivider(1000.0);

    gdr::CameraRGBD kinectCamera(517.3, 318.6, 516.5, 255.3);
    kinectCamera.setDepthPixelDivider(5000.0);

    std::string assocFile = "assoc.txt";

    double rotErrorTreshold = 0.04;
    double translationErrorTreshold = 0.04;

    double minCoefficientOfBiggestComponent = 0.5;
    double coefficientR = 1.2;
    double coefficientT = 1.2;

//    auto results = test::TesterReconstruction::testReconstruction("../../data/desk1_sampled_98_6",
//                       0.04, 0.04,
//                       kinectCamera,
//                       paramsRansacDefault,
//                       assocFile);
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

    ASSERT_LE(meanErrorL2BA, translationErrorTreshold);
    ASSERT_LE(meanErrorL2IRLS, translationErrorTreshold);
    ASSERT_LE(meanErrorRotBA, rotErrorTreshold);
    ASSERT_LE(meanErrorRotIRLS, rotErrorTreshold);

    ASSERT_LE(meanErrorRotBA, meanErrorRotIRLS * coefficientR);
    ASSERT_LE(meanErrorL2BA, meanErrorL2IRLS * coefficientT);

}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}



