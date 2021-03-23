//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ESTIMATORROBUSTLORANSAC_H
#define GDR_ESTIMATORROBUSTLORANSAC_H

#include "IEstimatorRelativePoseRobust.h"
#include "Estimator3Points.h"
#include "EstimatorNPoints.h"
#include "InlierCounter.h"

namespace gdr {

    class EstimatorRobustLoRANSAC : public IEstimatorRelativePoseRobust {
    public:
        SE3 estimateRelativePose(
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntrToBeTransformed,
                const CameraRGBD &cameraIntrDestination,
                const ParamsRANSAC &paramsRansac,
                bool& estimationSuccess,
                std::vector<int>& inlierIndices) override;

        virtual SE3
        getTransformationMatrixUmeyamaLoRANSACProjectiveError(
                const Estimator3Points& estimator3p,
                const EstimatorNPoints& estimatorNp,
                const InlierCounter& inlierCounter,
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntrToBeTransformed,
                const CameraRGBD &cameraIntrDestination,
                const ParamsRANSAC &paramsRansac,
                bool &estimationSuccess,
                std::vector<int>& inlierIndices) const;
    };
}

#endif