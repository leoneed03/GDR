//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_READERBUNDLEFUSION_H
#define GDR_READERBUNDLEFUSION_H

#include <cstring>
#include <boost/filesystem.hpp>
#include "parametrization/SE3.h"

namespace gdr {
    namespace fs = boost::filesystem;
    class ReaderBundleFusion {
        std::string pathToBundleFusionDataset;
        std::string pathToSaveLikeTumDataset;
        std::string rgbDir = "rgb";
        std::string depthDir = "depth";
    public:

        enum class IMAGE_TYPES {RGB, DEPTH, POSE, OTHER};

        IMAGE_TYPES getImageType(const std::string &fullPath);

        SE3 getGroundTruthPose(const std::string &fileName);

        std::string getTime(const std::string &imageName);

        void createFileTxt(const std::vector<std::string> &pathsToImages,
                           const std::string &imageType);

        ReaderBundleFusion(const std::string &pathToBundleFusionDatasetToSet,
                           const std::string &pathToSaveLikeTumToSet);

        void save(int maxNumberOfImages);

    };
}

#endif
