//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "quaternions.h"


namespace gdr {

//    std::vector<Eigen::Matrix3d> getRotationsFromQuaternionVector(const std::vector<std::vector<double>> &quats) {
//
//        std::vector<Eigen::Matrix3d> resultMatrices;
//        int count0 = 0;
//        for (const auto &quat: quats) {
//            ++count0;
//            Eigen::Quaterniond quatd(quat.data());
//            resultMatrices.push_back(quatd.toRotationMatrix());
//        }
//        return resultMatrices;
//    }

    void
    rotationOperations::applyRotationToAllFromLeft(std::vector<Eigen::Quaterniond> &orientations, Eigen::Quaterniond rotation) {
        for (auto& orientation: orientations) {
            orientation = (rotation * orientation).normalized();
        }
    }
}