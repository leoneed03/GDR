//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSEMEASUREMENT_H
#define GDR_RELATIVEPOSEMEASUREMENT_H

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"
#include "absolutePoseEstimation/translationAveraging/TranslationMeasurement.h"

namespace gdr {

    // TODO: is a duplicate of RelativePoseSE3

    class RelativePoseMeasurement {

        RotationMeasurement relativeRotation;

        TranslationMeasurement relativeTranslation;

    public:

        RelativePoseMeasurement(const RotationMeasurement &newRotationMeasurement,
                                const TranslationMeasurement &newTranslationMeasurement);

        Eigen::Quaterniond getRotationRelative() const;

        Eigen::Vector3d getTranslationRelative() const;

        int getIndexFromToBeTransformed() const;

        int getIndexToDestination() const;

    };
}

#endif
