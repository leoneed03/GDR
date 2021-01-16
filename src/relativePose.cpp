//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePose.h"

namespace gdr {
    relativePose::relativePose(const gdr::rotationMeasurement &newRotationMeasurement,
                               const gdr::translationMeasurement &newTranslationMeasurement,
                               int newIndexFrom,
                               int newIndexTo) : relativeRotation(newRotationMeasurement),
                                                 relativeTranslation(newTranslationMeasurement),
                                                 indexTo(newIndexTo),
                                                 indexFrom(newIndexFrom) {}
}
