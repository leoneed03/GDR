//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSEMEASUREMENT_H
#define GDR_RELATIVEPOSEMEASUREMENT_H

#include "absolutePoseEstimation/rotationAveraging/rotationMeasurement.h"
#include "absolutePoseEstimation/translationAveraging/translationMeasurement.h"

namespace gdr {

    // TODO: is a duplicate of RelativePoseSE3

    class RelativePoseMeasurement {

        rotationMeasurement relativeRotation;

        translationMeasurement relativeTranslation;

    public:

        RelativePoseMeasurement(const rotationMeasurement &newRotationMeasurement,
                                const translationMeasurement &newTranslationMeasurement);

        Eigen::Quaterniond getRotationRelative() const;

        Eigen::Vector3d getTranslationRelative() const;

        int getIndexFromToBeTransformed() const;

        int getIndexToDestination() const;

    };
}

#endif
