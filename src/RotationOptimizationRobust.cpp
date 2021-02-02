//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//



#include "RotationOptimizationRobust.h"

namespace gdr {

    RotationOptimizer::RotationOptimizer(const std::vector<Rotation3d> &newOrientations,
                                         const std::vector<rotationMeasurement> &pairWiseRotationsVector) :
            orientations(newOrientations),
            relativeRotations(pairWiseRotationsVector) {
    }

    std::vector<Eigen::Quaterniond> RotationOptimizer::getOptimizedOrientation(int indexFixed) const {

        int dim = 4;
        std::vector<std::vector<double>> result(orientations.size());
        for (int i = 0; i < orientations.size(); ++i) {
            Eigen::Quaterniond quat = orientations[i].getUnitQuaternion();
            result[i] = {quat.x(), quat.y(), quat.z(), quat.w()};
        }

        for (int i = 0; i < result.size(); ++i) {
            std::cout << i << ": \t POSE_Quat ";
            for (const auto& e: result[i]) {
                std::cout << e << ' ';
            }
            std::cout << std::endl;
        }



        ceres::Problem problem;
        ceres::LocalParameterization* quaternion_local_parameterization =
                new ceres::EigenQuaternionParameterization;
        for (const auto &relativeRotObservation: relativeRotations) {

            Eigen::Quaterniond quat = relativeRotObservation.getRotationQuat();
            std::vector<double> quatVector = {quat.x(), quat.y(), quat.z(), quat.w()};
            assert(result[relativeRotObservation.getIndexToDestination()].size() == dim);
            ceres::CostFunction *cost_function =
                    SnavelyReprojectionError::Create(quatVector);
            int indexFrom = relativeRotObservation.getIndexFromToBeTransformed();
            int indexTo = relativeRotObservation.getIndexToDestination();
            assert(indexFrom >= 0 && indexFrom < result.size() && indexTo >= 0 && indexTo < result.size());
            assert(result[indexFrom].size() == dim && result[indexTo].size() == dim);
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
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << "done ceres" << std::endl;
        std::cout << summary.FullReport() << "\n";



        for (int i = 0; i < result.size(); ++i) {
            std::cout << i << ": \t optimized_POSE_Quat ";
            for (const auto& e: result[i]) {
                std::cout << e << ' ';
            }
            std::cout << std::endl;
        }

        std::vector<Eigen::Quaterniond> optimizedOrientations;
        for (const auto& orientationVectorRaw: result) {
            Eigen::Quaterniond quat = Eigen::Quaterniond(orientationVectorRaw.data());
            optimizedOrientations.push_back(quat);
        }

        return optimizedOrientations;
    }
}
