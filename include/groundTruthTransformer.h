//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_GROUNDTRUTHTRANSFORMER_H
#define GDR_GROUNDTRUTHTRANSFORMER_H

#include <string>
#include <set>
#include <unordered_map>
#include <map>

#include "fileProc.h"
#include "poseInfo.h"
#include "quaternions.h"
#include "relativePose.h"

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gdr {

    void putAligned(std::ofstream &of, const std::vector<double> &val);

    std::vector<std::string> readData(std::string pathToRGB);

    struct GTT {
        enum class EXIT_CODES_CLASS {OK, NOT_A_DIRECTORY_ERROR, INVALID_DIRECTORY_PATH, FILE_DOES_NOT_EXIST};

        static const int infoElements = 8;

        static EXIT_CODES_CLASS createDepthTxtFile(const std::string &pathToRgbdDirectory,
                                                   const std::string &tobeCreatedFilenName);

        static std::vector<double> createTimestamps(const std::vector<std::string> &rgb, const std::string &pathTimeRGB,
                                                    const std::string &pathToGroundTruth, const std::set<int> &indices);

        static std::vector<std::vector<double>>
        getGroundTruth(const std::string &pathToGroundTruth, const std::vector<double> &timeStamps);

        static int
        makeRotationsRelative(const std::string &pathToGroundTruth, const std::string &pathToRelativeGroundTruth);

        static int
        writeGroundTruth(const std::string &pathOut, const std::vector<std::vector<double>> &timeCoordinates);

        static std::pair<std::vector<std::string>, std::vector<std::string>>
        makeRotationsRelativeAndExtractImages(const std::string &pathToGroundTruth, const std::string &pathToRGB,
                                              const std::string &pathToD, const std::string &pathOutDirectory,
                                              const std::string &timeInfo, const std::set<int> &indices);

        static void
        writeInfo(const std::vector<std::string> &rgb,
                  const std::string &pathTimeRGB,
                  const std::string &pathToGroundTruth,
                  const std::string &pathOut,
                  const std::string &rel,
                  const std::set<int> &indices);

        static void
        prepareDataset(const std::string &pathToDataset,
                       const std::string &pathOut,
                       const std::set<int> &chosenIndices,
                       const std::string &NewName);

        static int writeGroundTruthRelativeToZeroPose(const std::string &pathOut,
                                                      const std::vector<std::vector<double>> &timeCoordinates);

        static std::vector<std::vector<double>> extractTimeAndTransformation(const std::string &inputFileName);

        static std::vector<relativePose>
        extractAllRelativeTransformationPairwise(const std::string &in, const std::string &out,
                                                 std::string noise = "");

        static std::vector<relativePose> extractAllRelativeTransformationPairwise(const std::string &in);

        static std::vector<relativePose> readRelativePoses(const std::string &in);

        static std::vector<poseInfo> getPoseInfoTimeTranslationOrientation(const std::string &pathToGroundTruthFile);

        static int printRelativePosesFile(const std::vector<relativePose> &relativePoses,
                                          const std::string &pathOutRelativePoseFile, int numAbsPoses);
    };
}

#endif
