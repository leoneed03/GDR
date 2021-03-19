//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_FILES_H
#define GDR_FILES_H

#include <iostream>
#include <vector>
#include <dirent.h>
#include <memory>
#include <boost/timer.hpp>

#include <cassert>


namespace gdr {

    class DirectoryReader {
    public:
        static std::vector<std::string> readPathsToImagesFromDirectory(const std::string &pathToDirectory);
    };
}

#endif
