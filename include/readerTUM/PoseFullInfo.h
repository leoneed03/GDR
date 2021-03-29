//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POSEFULLINFO_H
#define GDR_POSEFULLINFO_H

#include <Eigen/Eigen>
#include <sophus/se3.hpp>

#include <iostream>

namespace gdr {

    struct PoseFullInfo {

        int elementsRaw = 8;
        double timestamp;
        //TODO: use SE3 class
        Eigen::Quaterniond orientationQuat;
        Eigen::Vector3d coordinated3d;


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PoseFullInfo(double newTimestamp,
                     const Eigen::Quaterniond &newOrientationQuat,
                     const Eigen::Vector3d &newCoordinates);

        explicit PoseFullInfo(const std::vector<double> &rawPoseInfoTimestampTranslationOrientation);

        Eigen::Quaterniond getOrientationQuat() const;

        Eigen::Vector3d getTranslation() const;

        Sophus::SE3d getSophusPose() const;

        double getTimestamp() const;

        friend std::ostream &operator<<(std::ostream &os, const PoseFullInfo &poseFullInfo);
    };
}

#endif
