//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IMAGESASSOCIATOR_H
#define GDR_IMAGESASSOCIATOR_H

#include <vector>
#include <string>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <poseInfo.h>

namespace gdr {
    class ImageAssociator {

        std::string pathToDatasetRoot;
        std::vector<std::string> imagesPathsRGB;
        std::vector<std::string> imagesPathsD;

        std::map<std::string, double> timestampByImageNameRGB;
        std::map<std::string, double> timestampByImageNameD;
        std::map<double, std::string> imageNamByTimestampRGB;
        std::map<double, std::string> imageNamByTimestampD;

        std::vector<std::pair<double, double>> associatedTimestampsRGBAndDepth;
        std::vector<std::pair<std::string, std::string>> associatedImagesRGBAndDepth;
        std::vector<poseInfo> associatedGroundTruthInfo;

//        std::map<std::string, int> imageNumberByNameRGB;
//        std::map<std::string, int> imageNumberByNameD;
        std::unordered_map<double, std::vector<double>> absolutePoseByTimestamp;

        /*
         * @param pathToDirectory -- path to the Directory with the images
         */

        static std::map<std::string, double> getTimestampsByImageName(const std::string &pathToDirectory);

        static std::map<std::string, poseInfo> getPoseInfoByImageName(const std::string &pathToDirectory);

        int checkSize() const;
    public:

        /*
         * @param pathToDirectory -- path to the Directory with all the data
         * In the Directory:
         * Provide rgb, depth directories with images
         * depth.txt and rgb.txt with images timestamps
         * groundtruth.txt with absolute poses (optionally)
         */

        ImageAssociator(const std::string &pathToDirectory,
                        const std::string &directoryNameRGB = "rgb",
                        const std::string &directoryNameD = "depth",
                        const std::string &groundtruthFileName = "groundtruth.txt");

        /*
         * @param outPutDirectoryPath -- path to the Directory where associated data will be saved
         * @param maxTimeTreshold -- max time difference between rgb and depth frames to be matched
         * @param timeOffset -- time offset to be added to all depth timestamps
         */

        int AssociateImagePairs(double maxTimeTreshold = 0.02,
                                double timeOffset = 0.0,
                                const std::string &rgbDirShortName = "rgb",
                                const std::string &depthDirShortName = "depth",
                                const std::string &groundtruthFileName = "groundtruth.txt",
                                const std::string &fileExtension = "txt");

        static std::vector<poseInfo> getAssociatedPoseInfo(
                const std::string &pathToGT,
                const std::vector<double> &timestamps,
                std::unordered_set<double> &foundTimestamps,
                double maxTimeTreshold = 0.02,
                double timeOffset = 0.0);

        int exportAllInfoToDirectory(const std::string &outDirectory,
                                     const std::set<int>& indicesToSample = {},
                                     bool exportGT = true,
                                     const std::string &shortNameRGBDir = "rgb",
                                     const std::string &shortNameDepthDir = "depth",
                                     const std::string &shortFilenameGT = "groundtruth",
                                     const std::string &extension = "txt"
        ) const;


    };
}

#endif
