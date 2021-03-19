//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_TRANSLATIONMEASUREMENT_H
#define GDR_TRANSLATIONMEASUREMENT_H

#include <Eigen/Eigen>

namespace gdr {

    class translationMeasurement {

        Eigen::Vector3d translation3d;

        int indexFrom;
        int indexTo;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        translationMeasurement(const Eigen::Vector3d &newTranslation3d, int newIndexFrom, int newIndexTo);

        const Eigen::Vector3d& getTranslation() const;

        int getIndexFromToBeTransformed() const;

        int getIndexToDestination() const;
    };
}
#endif
