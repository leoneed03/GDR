//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H
#define TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H

#include <string>
#include <set>
#include <unordered_map>
#include <map>

#include "fileProc.h"
#include "quaternions.h"


#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

struct GTT {
    static std::vector<double> createTimestamps(const std::vector<std::string> &rgb, const std::string &pathTimeRGB,
                                                const std::string &pathToGroundTruth, const std::set<int> &indices);

    static std::vector<std::vector<double>>
    getGroundTruth(const std::string &pathToGroundTruth, const std::vector<double> &timeStamps);

    static int
    makeRotationsRelative(const std::string &pathToGroundTruth, const std::string &pathToRelativeGroundTruth);

    static int writeGroundTruth(const std::string &pathOut, const std::vector<std::vector<double>> &timeCoordinates);

    static std::pair<std::vector<std::string>, std::vector<std::string>>
    makeRotationsRelativeAndExtractImages(const std::string &pathToGroundTruth, const std::string &pathToRGB,
                                          const std::string &pathToD, const std::string &pathOutDirectory,
                                          const std::string &timeInfo, const std::set<int> indices);

    static void
    writeInfo(const std::vector<std::string> &rgb, const std::string &pathTimeRGB, const std::string &pathToGroundTruth,
              const std::string &pathOut, const std::string &rel, const std::set<int> &indices);

    static void
    prepareDataset(const std::string &pathToDataset, const std::string &pathOut, const std::set<int> &chosenIndices,
                   const std::string NewName);

    static int writeGroundTruthRelativeToZeroPose(const std::string &pathOut,
                                                  const std::vector<std::vector<double>> &timeCoordinates);
};

#endif
