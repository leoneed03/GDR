//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RelativePosesG2oFormat.h"

namespace gdr {

    RelativePosesG2oFormat::RelativePosesG2oFormat(const std::vector<RotationMeasurement> &relativeRotationsToSet) :
            relativeRotations(relativeRotationsToSet) {}

    std::ostream &operator<<(std::ostream &os, const RelativePosesG2oFormat &rotationsG2o) {

        assert(!rotationsG2o.relativeRotations.empty());

        int minIndex = std::numeric_limits<int>::max() / 2;
        int maxIndex = -1;

        for (const auto &relativeRotation: rotationsG2o.relativeRotations) {
            int indexFromDestination = relativeRotation.getIndexFromDestination();
            int indexToToBeTransformed = relativeRotation.getIndexToToBeTransformed();

            if (indexFromDestination >= indexToToBeTransformed) {
                continue;
            }

            minIndex = std::min(minIndex, indexFromDestination);
            maxIndex = std::max(maxIndex, indexToToBeTransformed);
        }

        assert(minIndex == 0);
        assert(minIndex < maxIndex);


        for (int i = 0; i <= maxIndex; ++i) {
            std::string s1 = "VERTEX_SE3:QUAT ";
            std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
            os << s1 + s2;
        }

        std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

        for (const auto &relativeRotation: rotationsG2o.relativeRotations) {
            int indexFromDestination = relativeRotation.getIndexFromDestination();
            int indexToToBeTransformed = relativeRotation.getIndexToToBeTransformed();

            if (indexFromDestination >= indexToToBeTransformed) {
                continue;
            }

            os << "EDGE_SE3:QUAT " << indexFromDestination << ' ' << indexToToBeTransformed << ' ';

            os << 0.0 << ' ' << 0.0 << ' ' << 0.0 << ' ' << relativeRotation.getRotationSO3() << ' ';
            os << noise << std::endl;
        }

        return os;
    }
}