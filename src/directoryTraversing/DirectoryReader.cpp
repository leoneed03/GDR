//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <algorithm>
#include "boost/filesystem.hpp"

#include "directoryTraversing/DirectoryReader.h"

namespace gdr {

    namespace fs = boost::filesystem;

    std::vector<std::string> DirectoryReader::readPathsToImagesFromDirectorySorted(const std::string &pathToDirectory) {

        std::vector<std::string> images;
        fs::path imagesDirectory(pathToDirectory);

        for (const auto &imagesIt: fs::directory_iterator(imagesDirectory)) {
            images.emplace_back(imagesIt.path().string());
        }

        std::sort(images.begin(), images.end());

        return images;
    }

    std::string DirectoryReader::appendPathSuffix(const std::string &pathString, const std::string &suffixString) {
        fs::path path(pathString);

        path.append(suffixString);
        return path.string();
    }

}
