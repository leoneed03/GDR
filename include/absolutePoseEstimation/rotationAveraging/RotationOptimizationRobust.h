//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONOPTIMIZATIONROBUST_H
#define GDR_ROTATIONOPTIMIZATIONROBUST_H

#include <map>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "parametrization/SO3.h"
#include "rotationMeasurement.h"

namespace gdr {

    struct relativeRotationError {

        relativeRotationError(const std::vector<double> & relativeRotationToSet) {
            relativeRotation = relativeRotationToSet;
        }

        template <typename T>
        bool operator()(const T* const rotFrom,
                        const T* const rotTo,
                        T* residuals) const {

            std::vector<T> qRawFrom = {rotFrom[0], rotFrom[1], rotFrom[2], rotFrom[3]};
            std::vector<T> qRawTo = {rotTo[0], rotTo[1], rotTo[2], rotTo[3]};

            Eigen::Map<const Eigen::Quaternion<T>> qFrom(qRawFrom.data());
            Eigen::Map<const Eigen::Quaternion<T>> qTo(qRawTo.data());

            Eigen::Quaternion<T> qFromQuat(qRawFrom.data());

            Eigen::Quaternion<T> qToQuat(qRawTo.data());


            Eigen::Quaternion<T> relativeRotationComputed = qFrom.inverse() * qTo;

            std::vector<T> rawRelRotObserved = {T(relativeRotation[0]), T(relativeRotation[1]), T(relativeRotation[2]), T(relativeRotation[3])};
            Eigen::Map<const Eigen::Quaternion<T>> qMeasured(rawRelRotObserved.data());
            Eigen::Quaternion<T> qMeasuredQuat(rawRelRotObserved.data());
            qMeasuredQuat.normalize();

            Eigen::Quaternion<T> delta_q = qMeasuredQuat * relativeRotationComputed.inverse();
            Sophus::SO3<T> quatErrorSophus(delta_q.normalized().toRotationMatrix());
            Eigen::Map<Eigen::Matrix<T, 3, 1>> residualsM(residuals);
            auto logError = quatErrorSophus.log();

            residualsM.template block<3, 1>(0, 0) = T(2.0) * logError;

            return true;
        }

        static ceres::CostFunction* Create(const std::vector<double> & newRelativeRotation) {
            return (new ceres::AutoDiffCostFunction<relativeRotationError, 3, 4, 4>(
                    new relativeRotationError(newRelativeRotation)));
        }

        std::vector<double> relativeRotation;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class RotationOptimizer {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::vector<SO3> orientations;
        std::vector<rotationMeasurement> relativeRotations;

    public:

        RotationOptimizer(const std::vector<SO3>& orientations, const std::vector<rotationMeasurement>& pairWiseRotations);

        std::vector<SO3> getOptimizedOrientation(int indexFixed = 0) const;
    };
}

#endif
