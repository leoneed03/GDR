//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "translationMeasurement.h"

namespace gdr {

    translationMeasurement::translationMeasurement(const Eigen::Vector3d &newTranslation3d, int newIndexFrom,
                                                   int newIndexTo) : translation3d(newTranslation3d),
                                                                     indexFrom(newIndexFrom),
                                                                     indexTo(newIndexTo) {}
}

