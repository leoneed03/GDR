//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/rotationAveraging.h"
#include <gtsam/base/timing.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/InitializePose.h>
#include <gtsam/slam/dataset.h>

#include <boost/program_options.hpp>
#include <random>

using namespace gtsam;
namespace po = boost::program_options;

int rotationAverager::shanonAveraging(const std::string &pathToRelativeRotations, const std::string &pathOut) {
    std::string datasetName;
    std::string inputFile = pathToRelativeRotations;
    std::string outputFile = pathOut;
    int d = 3;
    srand(time(nullptr));
    int seed = rand();

    static std::mt19937 rng(seed);

    NonlinearFactorGraph::shared_ptr inputGraph;
    Values::shared_ptr posesInFile;
    Values poses;
    if (d == 3) {
        std::cout << "Running Shonan averaging for SO(3) on " << inputFile << endl;
        ShonanAveraging3 shonan(inputFile);
        auto initial = shonan.initializeRandomly(rng);
        auto result = shonan.run(initial);

        // Parse file again to set up translation problem, adding a prior
        boost::tie(inputGraph, posesInFile) = load3D(inputFile);
        auto priorModel = noiseModel::Unit::Create(6);
        inputGraph->addPrior(0, posesInFile->at<Pose3>(0), priorModel);

        std::cout << "recovering 3D translations" << endl;
        auto poseGraph = initialize::buildPoseGraph<Pose3>(*inputGraph);
        poses = initialize::computePoses<Pose3>(result.first, &poseGraph);
    }
    std::cout << "Writing result to " << outputFile << endl;
    writeG2o(NonlinearFactorGraph(), poses, outputFile);
    return 0;
}
