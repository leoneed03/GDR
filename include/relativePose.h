//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSE_H
#define GDR_RELATIVEPOSE_H

#include "absolutePoseEstimation/rotationAveraging/rotationMeasurement.h"
#include "absolutePoseEstimation/translationAveraging/translationMeasurement.h"

namespace gdr {
    struct relativePose {

        rotationMeasurement relativeRotation;

        rotationMeasurement getRelativeRotation() const;

        translationMeasurement getRelativeTranslation() const;

        translationMeasurement relativeTranslation;


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        relativePose(const rotationMeasurement &newRotationMeasurement,
                     const translationMeasurement &newTranslationMeasurement);

        Eigen::Quaterniond getRotationRelative() const;

        Eigen::Vector3d getTranslationRelative() const;

        int getIndexFromToBeTransformed() const;

        int getIndexToDestination() const;

    };
}

#endif
