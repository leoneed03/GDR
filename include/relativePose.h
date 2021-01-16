//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSE_H
#define GDR_RELATIVEPOSE_H

#include "rotationMeasurement.h"
#include "translationMeasurement.h"

namespace gdr {
    struct relativePose {
        rotationMeasurement relativeRotation;
        translationMeasurement relativeTranslation;
        int indexFrom;
        int indexTo;
        relativePose(const rotationMeasurement& newRotationMeasurement, const translationMeasurement& newTranslationMeasurement,
                     int newIndexFrom, int newIndexTo);
    };
}

#endif
