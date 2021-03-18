//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONOPTIMIZATIONROBUST_H
#define GDR_ROTATIONOPTIMIZATIONROBUST_H

#include <map>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "parametrization/Rotation3d.h"
#include "rotationMeasurement.h"

namespace gdr {

    struct SnavelyReprojectionError {
//        SnavelyReprojectionError(const Eigen::Quaterniond & newRelativeRotation) {
//
//            relativeRotation = {newRelativeRotation.x(), newRelativeRotation.y(),
//                                newRelativeRotation.z(), newRelativeRotation.w()};
//        }
        SnavelyReprojectionError(const std::vector<double> & newRelativeRotation) {

            relativeRotation = newRelativeRotation;
        }

        template <typename T>
        bool operator()(const T* const rotFrom,
                        const T* const rotTo,
                        T* residuals) const {


//            std::cout << "init raw vectors" << std::endl;
            std::vector<T> qRawFrom = {rotFrom[0], rotFrom[1], rotFrom[2], rotFrom[3]};
            std::vector<T> qRawTo = {rotTo[0], rotTo[1], rotTo[2], rotTo[3]};
//            std::cout << "init raw vectors end" << std::endl;
            Eigen::Map<const Eigen::Quaternion<T>> qFrom(qRawFrom.data());
            Eigen::Map<const Eigen::Quaternion<T>> qTo(qRawTo.data());

            Eigen::Quaternion<T> qFromQuat(qRawFrom.data());

            Eigen::Quaternion<T> qToQuat(qRawTo.data());

//            qFromQuat.normalize();
//            qToQuat.normalize();


            Eigen::Quaternion<T> relativeRotationComputed = qFrom.inverse() * qTo;

//////            Eigen::Quaternion<T> relativeRotationComputed = qTo.conjugate() * qFrom;
//            std::vector<T> rawRelRotObserved = {T(relativeRotation[0]), T(relativeRotation[1]), T(relativeRotation[2]), T(relativeRotation[3])};
//
//            Eigen::Quaternion<T> qObserved(rawRelRotObserved.data());
//            Eigen::Quaternion<T> quatError = (relativeRotationComputed.inverse().normalized() * qObserved).normalized();

//
//            std::vector<double> qMeasured = relativeRotation;
//            Eigen::Map<const Eigen::Quaternion<double>> qMeasuredQuat(qMeasured.data());
//            Eigen::Quaternion<T> delta_q =
//                    qMeasuredQuat.template cast<T>() * relativeRotationComputed.conjugate();
//
//
//            Eigen::Map<Eigen::Matrix<T, 3, 1>> residualsM(residuals);
//            residualsM.template block<3, 1>(0, 0) = T(2.0) * delta_q.vec();

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

/*
        template <typename T>
        bool operator()(const T* const rotFrom,
                        const T* const rotTo,
                        T* residuals) const {


//            std::cout << "init raw vectors" << std::endl;
            std::vector<T> qRawFrom = {rotFrom[3], rotFrom[0], rotFrom[1], rotFrom[2]};
            std::vector<T> qRawTo = {rotTo[3], rotTo[0], rotTo[1], rotTo[2]};

            std::vector<T> qRawToInverse = {qRawTo[0], -qRawTo[1], -qRawTo[2], -qRawTo[3]};
            T norm = T(0);
            for (const auto& e: qRawTo) {
                norm += e * e;
            }
            norm = sqrt(norm);
            for (auto& e: qRawTo) {
                e /= norm;
            }

            std::vector<T> estimatedDifference(4);
            ceres::QuaternionProduct(qRawToInverse.data(), qRawFrom.data(), estimatedDifference.data());
            residuals[0] = estimatedDifference[1];
            residuals[1] = estimatedDifference[2];
            residuals[2] = estimatedDifference[3];

            return true;
        }*/

        // Factory to hide the construction of the CostFunction object from
        // the client code.
//        static ceres::CostFunction* Create(const Eigen::Quaterniond & newRelativeRotation) {
//            return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 3, 4, 4>(
//                    new SnavelyReprojectionError(newRelativeRotation)));
//        }

        static ceres::CostFunction* Create(const std::vector<double> & newRelativeRotation) {
            return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 3, 4, 4>(
                    new SnavelyReprojectionError(newRelativeRotation)));
        }

        std::vector<double> relativeRotation;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct RotationOptimizer {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::vector<Rotation3d> orientations;
//        std::vector<std::map<int, Rotation3d>> pairWiseRotations;
        std::vector<rotationMeasurement> relativeRotations;

        RotationOptimizer(const std::vector<Rotation3d>& newOrientations, const std::vector<rotationMeasurement>& pairWiseRotations);

        std::vector<Eigen::Quaterniond> getOptimizedOrientation(int indexFixed = 0) const;
    };
}

#endif
