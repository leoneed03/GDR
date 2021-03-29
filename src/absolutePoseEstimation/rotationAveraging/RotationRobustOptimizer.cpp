//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//



#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.h"

namespace gdr {

    RotationRobustOptimizer::RotationRobustOptimizer(const std::vector<SO3> &newOrientations,
                                                     const std::vector<RotationMeasurement> &pairWiseRotationsVector) :
            orientations(newOrientations),
            relativeRotations(pairWiseRotationsVector) {
    }

    std::vector<SO3> RotationRobustOptimizer::getOptimizedOrientation(int indexFixed,
                                                                      bool printProgressToConsole) const {

        int dim = 4;
        std::vector<std::vector<double>> result(orientations.size());
        for (int i = 0; i < orientations.size(); ++i) {
            Eigen::Quaterniond quat = orientations[i].getUnitQuaternion();
            result[i] = {quat.x(), quat.y(), quat.z(), quat.w()};
        }

        ceres::Problem problem;
        ceres::LocalParameterization* quaternion_local_parameterization =
                new ceres::EigenQuaternionParameterization;
        for (const auto &relativeRotObservation: relativeRotations) {

            Eigen::Quaterniond quat = relativeRotObservation.getRotationQuat();
            std::vector<double> quatVector = {quat.x(), quat.y(), quat.z(), quat.w()};
            assert(result[relativeRotObservation.getIndexToDestination()].size() == dim);
            ceres::CostFunction *cost_function = relativeRotationError::Create(quatVector);
            int indexFrom = relativeRotObservation.getIndexFromToBeTransformed();
            int indexTo = relativeRotObservation.getIndexToDestination();
            assert(indexFrom >= 0 && indexFrom < result.size() && indexTo >= 0 && indexTo < result.size());
            assert(result[indexFrom].size() == dim && result[indexTo].size() == dim);

            // TODO: fix loss function
            problem.AddResidualBlock(cost_function,
                                     new ceres::CauchyLoss(0.5),
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
        for (const auto& orientationVectorRaw: result) {
            auto quat = Eigen::Quaterniond(orientationVectorRaw.data());
            optimizedOrientations.emplace_back(SO3(quat.normalized()));
        }

        return optimizedOrientations;
    }
}
