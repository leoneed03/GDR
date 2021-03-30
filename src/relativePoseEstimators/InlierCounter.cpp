//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseEstimators/InlierCounter.h"

namespace gdr {

    std::vector<std::pair<double, int>>
    InlierCounter::calculateInlierProjectionErrors(const Eigen::Matrix4Xd &toBeTransformedPoints,
                                                   const Eigen::Matrix4Xd &destinationPoints,
                                                   const CameraRGBD &cameraIntr3x3Destination,
                                                   const SE3 &umeyamaRt,
                                                   const ParamsRANSAC &paramsRansac) const {

        //TODO: add L2 error for inlier detection
        double thresholdProjectionErrorPixels = paramsRansac.getMaxProjectionErrorPixels();

        assert(toBeTransformedPoints.cols() == destinationPoints.cols());
        Eigen::Matrix4Xd pointsAfterTransformation = umeyamaRt.getSE3().matrix() * toBeTransformedPoints;

        std::vector<std::pair<double, int>> projectionErrorsInliers;
        assert(toBeTransformedPoints.cols() == pointsAfterTransformation.cols());

        for (int pointCounter = 0; pointCounter < pointsAfterTransformation.cols(); ++pointCounter) {
            Eigen::Vector3d toBeTransformedProjection =
                    cameraIntr3x3Destination.getIntrinsicsMatrix3x3() *
                    pointsAfterTransformation.col(pointCounter).topLeftCorner<3, 1>();
            Eigen::Vector3d destinationPointProjection =
                    cameraIntr3x3Destination.getIntrinsicsMatrix3x3() *
                    destinationPoints.col(pointCounter).topLeftCorner<3, 1>();

            for (int i = 0; i < 2; ++i) {
                toBeTransformedProjection[i] /= toBeTransformedProjection[2];
                destinationPointProjection[i] /= destinationPointProjection[2];
            }

            // TODO: depth error can be also checked
            Sophus::Vector2d projectionErrorPixels2d = (toBeTransformedProjection -
                                                        destinationPointProjection).topLeftCorner<2, 1>();

            double errorProjectionLp = 0;

            if (paramsRansac.getLpMetricParam() == 1) {
                errorProjectionLp = projectionErrorPixels2d.lpNorm<1>();
            } else if (paramsRansac.getLpMetricParam() == 2) {
                errorProjectionLp = projectionErrorPixels2d.lpNorm<2>();
            } else {
                assert(false && "only p=1 and p=2 L_p norms for reprojection error can be used");
            }

            if (errorProjectionLp < thresholdProjectionErrorPixels) {
                projectionErrorsInliers.emplace_back(std::make_pair(errorProjectionLp, pointCounter));
            }
        }

        return projectionErrorsInliers;
    }

}