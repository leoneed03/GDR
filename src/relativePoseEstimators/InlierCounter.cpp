//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseEstimators/InlierCounter.h"

namespace gdr {

    std::vector<std::pair<double, int>>
    InlierCounter::calculateProjectionErrors(const Eigen::Matrix4Xd &toBeTransformedPoints,
                                             const Eigen::Matrix4Xd &destinationPoints,
                                             const CameraRGBD &cameraDest,
                                             const SE3 &umeyamaRt,
                                             const ParamsRANSAC &paramsRansac) const {


        double maxProjectionErrorPixels = paramsRansac.getMaxProjectionErrorPixels();
        double max3DError = paramsRansac.getMax3DError();

        assert(toBeTransformedPoints.cols() == destinationPoints.cols());
        Eigen::Matrix4Xd pointsAfterTransformation = umeyamaRt.getSE3().matrix() * toBeTransformedPoints;

        std::vector<std::pair<double, int>> projectionErrorsInliers;
        assert(toBeTransformedPoints.cols() == pointsAfterTransformation.cols());

        for (int pointCounter = 0; pointCounter < pointsAfterTransformation.cols(); ++pointCounter) {
            Eigen::Vector3d toBeTransformedProjection =
                    cameraDest.getIntrinsicsMatrix3x3() *
                    pointsAfterTransformation.col(pointCounter).topLeftCorner<3, 1>();
            Eigen::Vector3d destinationPointProjection =
                    cameraDest.getIntrinsicsMatrix3x3() * destinationPoints.col(pointCounter).topLeftCorner<3, 1>();
            for (int i = 0; i < 2; ++i) {
                toBeTransformedProjection[i] /= toBeTransformedProjection[2];
                destinationPointProjection[i] /= destinationPointProjection[2];
            }

            // TODO: depth error can be also checked
            Eigen::Vector3d projectionErrorPixels2d = (toBeTransformedProjection - destinationPointProjection);

            double maxError = std::max(std::abs(projectionErrorPixels2d[0]), std::abs(projectionErrorPixels2d[1]));

            if (maxError < maxProjectionErrorPixels) {
                projectionErrorsInliers.emplace_back(std::make_pair(maxError, pointCounter));
            }
        }

        return projectionErrorsInliers;
    }

}