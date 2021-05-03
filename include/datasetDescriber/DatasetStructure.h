//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_DATASETSTRUCTURE_H
#define GDR_DATASETSTRUCTURE_H

#include <vector>
#include <string>

namespace gdr {

    struct DatasetStructure {
        std::vector<std::string> pathsImagesRgb;
        std::vector<std::string> pathsImagesDepth;
        std::vector<std::pair<double, double>> timestampsRgbDepth;
    };
}


#endif
