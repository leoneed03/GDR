//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_DIRECTORY_READER_H
#define GDR_DIRECTORY_READER_H

#include <vector>
#include <string>

namespace gdr {

    class DirectoryReader {
    public:
        static std::vector<std::string> readPathsToImagesFromDirectory(const std::string &pathToDirectory);
    };
}

#endif
