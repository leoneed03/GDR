//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "boost/math/constants/constants.hpp"

#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.h"

namespace gdr {

    std::vector<SO3> RotationRobustOptimizer::getOptimizedOrientation(const std::vector<SO3> &orientationsToSet,
                                                                      const std::vector<RotationMeasurement> &pairWiseRotationsToSet,
                                                                      int indexFixed) {

        orientations = orientationsToSet;
        relativeRotations = pairWiseRotationsToSet;

        int dim = 4;
        std::vector<std::vector<double>> result(orientations.size());

        for (int i = 0; i < orientations.size(); ++i) {
            Eigen::Quaterniond quat = orientations[i].getUnitQuaternion();
            result[i] = {quat.x(), quat.y(), quat.z(), quat.w()};
        }

        ceres::Problem problem;
        ceres::LocalParameterization *quaternion_local_parameterization =
                new ceres::EigenQuaternionParameterization;

        for (const auto &relativeRotObservation: relativeRotations) {

            assert(result[relativeRotObservation.getIndexToToBeTransformed()].size() == dim);
            ceres::CostFunction *cost_function = RelativeRotationError::Create(relativeRotObservation.getRotationSO3());

            int indexFrom = relativeRotObservation.getIndexFromDestination();
            int indexTo = relativeRotObservation.getIndexToToBeTransformed();

            assert(indexFrom >= 0 && indexFrom < result.size() && indexTo >= 0 && indexTo < result.size());
            assert(result[indexFrom].size() == dim && result[indexTo].size() == dim);

            double pi = boost::math::constants::pi<double>();
            double thresholdRotationError = pi / 180;

            problem.AddResidualBlock(cost_function,
                                     new ceres::CauchyLoss(thresholdRotationError),
                                     result[indexFrom].data(),
                                     result[indexTo].data());

            problem.SetParameterization(result[indexFrom].data(), quaternion_local_parameterization);
            problem.SetParameterization(result[indexTo].data(), quaternion_local_parameterization);
        }
        problem.SetParameterBlockConstant(result[indexFixed].data());


        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = printProgressToConsole;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (printProgressToConsole) {
            std::cout << "done ceres robust rotation optimization" << std::endl;
            std::cout << summary.FullReport() << "\n";
        }

        assert(summary.IsSolutionUsable());

        std::vector<SO3> optimizedOrientations;
        for (const auto &orientationVectorRaw: result) {
            auto quat = Eigen::Quaterniond(orientationVectorRaw.data());
            optimizedOrientations.emplace_back(SO3(quat.normalized()));
        }

        return optimizedOrientations;
    }

    bool RotationRobustOptimizer::getPrintToConsole() const {
        return printProgressToConsole;
    }

    void RotationRobustOptimizer::setPrintProgressToConsole(bool printToConsoleToSet) {
        printProgressToConsole = printToConsoleToSet;
    }
}
