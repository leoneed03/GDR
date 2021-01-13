//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "rotationAveraging.h"

#include <random>
#include <fstream>
#include <gtsam/base/timing.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/InitializePose.h>
//#include <gtsam/slam/dataset.h>

namespace gdr {

    int rotationAverager::shanonAveraging(const std::string &pathToRelativeRotations, const std::string &pathOut) {
        std::string inputFile = pathToRelativeRotations;

        // Seed random number generator
        int seed = 42;
        std::mt19937 rng(seed);

        gtsam::NonlinearFactorGraph::shared_ptr inputGraph;
        gtsam::Values::shared_ptr posesInFile;
        gtsam::Values poses;
        {
            std::cout << "Running Shonan averaging for SO(3) on " << inputFile << std::endl;
            gtsam::ShonanAveraging3 shonan(inputFile);
            auto initial = shonan.initializeRandomly(rng);
            auto result = shonan.run(initial);

            // Parse file again to set up translation problem, adding a prior
            boost::tie(inputGraph, posesInFile) = gtsam::load3D(inputFile);
            auto priorModel = gtsam::noiseModel::Unit::Create(6);
            inputGraph->addPrior(0, posesInFile->at<gtsam::Pose3>(0), priorModel);

            std::cout << "recovering 3D translations" << std::endl;
            auto poseGraph = gtsam::initialize::buildPoseGraph<gtsam::Pose3>(*inputGraph);
            poses = gtsam::initialize::computePoses<gtsam::Pose3>(result.first, &poseGraph);
        }
        std::cout << "Writing result to " << pathOut << std::endl;
        writeG2o(gtsam::NonlinearFactorGraph(), poses, pathOut);
        return 0;
    }
}