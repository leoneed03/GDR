//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_TESTERRECONSTRUCTION_H
#define GDR_TESTERRECONSTRUCTION_H

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <chrono>

#include "readerDataset/readerTUM/ReaderTum.h"

#include "computationHandlers/RelativePosesComputationHandler.h"
#include "computationHandlers/ModelCreationHandler.h"

#include "readerDataset/readerTUM/Evaluator.h"

#include "poseGraph/PosesForEvaluation.h"

namespace test {

    struct ErrorsOfTrajectoryEstimation {
        int numberOfPosesInDataset = 0;
        gdr::ErrorRotationTranslation errorIRLS;
        gdr::ErrorRotationTranslation errorBA;
        gdr::ErrorRotationTranslation errorAlignedUmeyamaIRLS;
        gdr::ErrorRotationTranslation errorAlignedUmeyamaBA;
    };

    struct OutputShortFileNames {
        std::string posesIRLS = "posesIRLS.txt";
        std::string posesBA = "posesBA.txt";
        std::string posesGroundTruth = "posesGT.txt";
    };

    struct TesterReconstruction {
        static ErrorsOfTrajectoryEstimation testReconstruction(
                const std::string &datasetName,
                const gdr::CameraRGBD &cameraDefault = gdr::CameraRGBD(),
                const gdr::ParamsRANSAC &paramsRansac = gdr::ParamsRANSAC(),
                const std::string &assocFile = "",
                const std::string &pathOutPoses = "",
                const OutputShortFileNames &outputShortFileNames = OutputShortFileNames(),
                double timeDiffThreshold = 0.02,
                bool printToConsole = false,
                bool showVisualization3D = false,
                bool savePointCloudPly = false,
                const std::vector<int> &gpuDevices = {0},
                bool printInfoReport = true);
    };
}


#endif
