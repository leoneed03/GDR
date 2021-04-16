//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEROTATIONERROR_H
#define GDR_RELATIVEROTATIONERROR_H

#include <vector>
#include <ceres/ceres.h>
#include <sophus/so3.hpp>

#include "parametrization/SO3.h"

namespace gdr {

    class RelativeRotationError {

    private:
        SO3 relativeRotation;

    public:
        RelativeRotationError(const SO3 &relativeRotationToSett);

        template<typename T>
        bool operator()(const T *const rotFrom,
                        const T *const rotTo,
                        T *residuals) const {

            Eigen::Map<const Eigen::Quaternion<T>> qFrom(rotFrom);
            Eigen::Map<const Eigen::Quaternion<T>> qTo(rotTo);

            Eigen::Quaternion<T> relativeRotationComputed = qFrom.inverse() * qTo;

            const auto &relativeRotationQuaternion = relativeRotation.getUnitQuaternion();
            std::vector<T> rawRelRotObserved = {T(relativeRotationQuaternion.x()),
                                                T(relativeRotationQuaternion.y()),
                                                T(relativeRotationQuaternion.z()),
                                                T(relativeRotationQuaternion.w())};

            Eigen::Map<const Eigen::Quaternion<T>> relRotMeasured(rawRelRotObserved.data());

            Eigen::Quaternion<T> relRotRes = relRotMeasured * relativeRotationComputed.inverse();

            Sophus::SO3<T> quatErrorSophus(relRotRes.normalized().toRotationMatrix());
            Eigen::Map<Eigen::Matrix<T, 3, 1>> residualsM(residuals);

            auto logError = quatErrorSophus.log();

            residualsM.template block<3, 1>(0, 0) = T(2.0) * logError;

            return true;
        }

        static ceres::CostFunction *Create(const SO3 &relativeRotation);

    };
}

#endif
