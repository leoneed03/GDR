//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"
#include "absolutePoseEstimation/rotationAveraging/RelativePosesG2oFormat.h"
#include <random>
#include <fstream>

#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/InitializePose.h>
#include <gtsam/slam/dataset.h>

namespace gdr {

    std::vector<SO3> RotationAverager::shanonAveraging(
            const std::vector<RotationMeasurement> &relativeRotations,
            const std::string &pathToRelativeRotationsOut,
            int indexPoseFixed,
            int maxDimension,
            bool printProgressToConsole) {


        std::ofstream outRelRotations(pathToRelativeRotationsOut);
        outRelRotations << RelativePosesG2oFormat(relativeRotations);

        std::vector<SO3> absoluteRotationsSO3;

        int seed = 42;
        std::mt19937 rng(seed);

        gtsam::NonlinearFactorGraph::shared_ptr inputGraph;
        gtsam::Values::shared_ptr posesInFile;
        gtsam::Values poses;
        {
            if (printProgressToConsole) {
                std::cout << "Shonan Averaging: " << pathToRelativeRotationsOut << std::endl;
            }

            gtsam::ShonanAveraging3 shonan(pathToRelativeRotationsOut);
            auto initial = shonan.initializeRandomly(rng);

            auto result = shonan.run(initial, 3, maxDimension);

            boost::tie(inputGraph, posesInFile) = gtsam::load3D(pathToRelativeRotationsOut);
            auto priorModel = gtsam::noiseModel::Unit::Create(6);
            inputGraph->addPrior(0, posesInFile->at<gtsam::Pose3>(0), priorModel);

            auto poseGraph = gtsam::initialize::buildPoseGraph<gtsam::Pose3>(*inputGraph);
            poses = gtsam::initialize::computePoses<gtsam::Pose3>(result.first, &poseGraph);
        }

        for (const auto key_value : poses) {
            auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(&key_value.value);
            if (!p) {
                continue;
            }
            const gtsam::Pose3 &pose = p->value();
            const auto q = pose.rotation().toQuaternion();
            absoluteRotationsSO3.emplace_back(SO3(q));
        }

        assert(indexPoseFixed >= 0 && indexPoseFixed < absoluteRotationsSO3.size());

        auto orientationZeroInversed = absoluteRotationsSO3[indexPoseFixed].inverse();

        for (auto &orientation: absoluteRotationsSO3) {
            orientation = orientationZeroInversed * orientation;
        }

        return absoluteRotationsSO3;
    }
}