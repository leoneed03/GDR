//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include "TesterReconstruction.h"

int main(int argc, char* argv[]) {

    std::cout << "input args format: [path Dataset] [pathOutPoses] [fx] [fy] [cx] [cy] [depthDivider], " <<
                 "optionally: [fileOutIRLS] [fileOutBA] [fileOutGT] : [GPU index]" << std::endl;
    std::cout << "your args is " << argc << std::endl;
    assert(argc == 8 || argc == 11 || argc == 12);

    std::string pathDatasetRoot(argv[1]);
    std::string pathOutPoses(argv[2]);
    std::vector<double> intrinsics;
    test::OutputShortFileNames outputShortFileNames;

    std::string& fileIRLS = outputShortFileNames.posesIRLS;
    std::string& fileBA = outputShortFileNames.posesBA;
    std::string& fileGT = outputShortFileNames.posesGroundTruth;

    double depthDivider = std::stod(std::string(argv[7]));

    int gpuIndex = 0;

    if (argc >= 12) {
        gpuIndex = std::stoi(std::string(argv[11]));
    }
    std::cout << "using " << gpuIndex << " gpu" << std::endl;

    for (int i = 3; i < 7; ++i) {
        intrinsics.emplace_back(std::stod(std::string(argv[i])));
    }

    assert(intrinsics.size() == 4);

    double fx = intrinsics[0];
    double fy = intrinsics[1];
    double cx = intrinsics[2];
    double cy = intrinsics[3];

    std::cout << "Running reconstruction on " << pathDatasetRoot << std::endl
    << "poses will be printed to " << pathOutPoses << std::endl
    << "fx, fy, cx, cy: " << fx << ' ' << fy << ' ' << cx << ' ' << cy << std::endl
    << "depth in pixel divider: " << depthDivider << std::endl;

    if (argc >= 11) {
        fileIRLS = std::string(argv[8]);
        fileBA = std::string(argv[9]);
        fileGT = std::string(argv[10]);

        outputShortFileNames.posesIRLS = fileIRLS;
        outputShortFileNames.posesBA = fileBA;
        outputShortFileNames.posesGroundTruth = fileGT;
    }

    std::cout << "fileIRLS: " << fileIRLS << std::endl
              << "fileBA: " << fileBA << std::endl
              << "fileGT: " << fileGT << std::endl;

    gdr::ParamsRANSAC paramsRansacDefault;
    paramsRansacDefault.setProjectionUsage(false);

    gdr::CameraRGBD kinectCamera(fx, cx, fy, cy);
    kinectCamera.setDepthPixelDivider(depthDivider);

    std::string assocFile = "assoc.txt";

    test::TesterReconstruction::testReconstruction(pathDatasetRoot,
                                                   kinectCamera,
                                                   paramsRansacDefault,
                                                   assocFile,
                                                   pathOutPoses,
                                                   outputShortFileNames,
                                                   0.02,
                                                   false,
                                                   false,
                                                   false,
                                                   {gpuIndex},
                                                   true);
    return 0;
}