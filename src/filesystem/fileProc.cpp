//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include "boost/filesystem.hpp"

#include "filesystem/fileProc.h"
#include "printer.h"

#include <algorithm>

namespace gdr {

    namespace fs = boost::filesystem;

    std::vector<std::string> DirectoryReader::extractPathsToImagesFromDirectory(const std::string& pathToDirectory) {

        std::vector<std::string> images;
        fs::path imagesDirectory(pathToDirectory);

        for (fs::directory_iterator imagesEndIt, imagesIt(imagesDirectory); imagesIt != imagesEndIt; ++imagesIt) {
            images.emplace_back(imagesIt->path().string());
        }

        std::sort(images.begin(), images.end());

        return images;
    }

}
