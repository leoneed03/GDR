//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POSEINFO_H
#define GDR_POSEINFO_H

#include <Eigen/Eigen>
#include <iostream>

namespace gdr {

    struct poseInfo {
        const int elementsRaw = 8;
        double timestamp;
        Eigen::Quaterniond orientationQuat;
        Eigen::Vector3d coordinated3d;

        poseInfo(double newTimestamp, const Eigen::Quaterniond &newOrientationQuat,
                 const Eigen::Vector3d &newCoordinates);

        poseInfo(const std::vector<double> &rawPoseInfoTimestampTranslationOrientation);

        friend std::ostream &operator<<(std::ostream &os, const poseInfo &dt);
    };
}

#endif
