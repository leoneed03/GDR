//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"

#include <random>
#include <fstream>

#include <gtsam/base/timing.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/InitializePose.h>

#include <gtsam/slam/dataset.h>

namespace gdr {

    std::vector<SO3> RotationAverager::shanonAveraging(const std::string &pathToRelativeRotationsInput,
                                                       const std::string &pathOut,
                                                       bool printProgressToConsole) {

        std::string inputFile = pathToRelativeRotationsInput;
        std::vector<SO3> absoluteRotationsSO3;

        int seed = 42;
        std::mt19937 rng(seed);

        gtsam::NonlinearFactorGraph::shared_ptr inputGraph;
        gtsam::Values::shared_ptr posesInFile;
        gtsam::Values poses;
        {
            if (printProgressToConsole) {
                std::cout << "Shonan Averaging: " << inputFile << std::endl;
            }
            gtsam::ShonanAveraging3 shonan(inputFile);
            auto initial = shonan.initializeRandomly(rng);
            auto result = shonan.run(initial);

            // Parse file again to set up translation problem, adding a prior
            boost::tie(inputGraph, posesInFile) = gtsam::load3D(inputFile);
            auto priorModel = gtsam::noiseModel::Unit::Create(6);
            inputGraph->addPrior(0, posesInFile->at<gtsam::Pose3>(0), priorModel);

            auto poseGraph = gtsam::initialize::buildPoseGraph<gtsam::Pose3>(*inputGraph);
            poses = gtsam::initialize::computePoses<gtsam::Pose3>(result.first, &poseGraph);
        }

        if (printProgressToConsole) {
            std::cout << "result to " << pathOut << std::endl;
        }
        writeG2o(gtsam::NonlinearFactorGraph(), poses, pathOut);

        for (const auto key_value : poses) {
            auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(&key_value.value);
            if (!p) {
                continue;
            }
            const gtsam::Pose3 &pose = p->value();
            const auto q = pose.rotation().toQuaternion();
            absoluteRotationsSO3.emplace_back(SO3(q));
        }

        return absoluteRotationsSO3;
    }
}