//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_UMEYAMA_H
#define GDR_UMEYAMA_H

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace gdr {

    std::vector<std::pair<double, int>> calculateProjectionErrors(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                                  const Eigen::Matrix4Xd &destinationPoints,
                                                                  const Eigen::Matrix3d &cameraIntr3x3Destination,
                                                                  const Eigen::Matrix4d &cR_t_umeyama,
                                                                  double maxProjectionErrorPixels);

    std::vector<std::pair<double, int>> getPartionedByNthElement(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                                 const Eigen::Matrix4Xd &destinationPoints,
                                                                 const Eigen::Matrix4d &cR_t,
                                                                 int numberOfSeparatorElement);

    Eigen::Matrix4d
    getTransformationMatrixUmeyamaLoRANSAC(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                           const Eigen::Matrix4Xd &destinationPoints,
                                           const int numIterationsRansac,
                                           const int numOfElements,
                                           double inlierCoeff,
                                           bool &estimationSuccess,
                                           double maxErrorCorrespondence);

    Eigen::Matrix4d
    getTransformationMatrixUmeyamaLoRANSACProjectiveError(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                          const Eigen::Matrix4Xd &destinationPoints,
                                                          const Eigen::Matrix3d &cameraIntr3x3ToBeTransformed,
                                                          const Eigen::Matrix3d &cameraIntr3x3Destination,
                                                          const int numIterationsRansac,
                                                          const int numOfElements,
                                                          double inlierCoeff,
                                                          bool &estimationSuccess,
                                                          double maxProjectionErrorPixels);
}

#endif
