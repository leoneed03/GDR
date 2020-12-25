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

    std::vector<std::string> readRgbData(std::string pathToRGB);

    std::vector<std::vector<double>> parseAbsoluteRotationsFile(const std::string &pathToRotationsFile);

    std::vector<std::vector<double>> parseAbsoluteRotationsFile(const std::string &pathToRotationsFile);

    std::vector<std::vector<std::vector<double>>> getMeasurements(const std::string &truePairwiseTransformation);

    std::vector<std::pair<double, double>> getErrorMeasurements(const std::string &truePairwiseTransformation,
                                                                const std::string &estimatedPairwiseTransformations);
}

#endif
