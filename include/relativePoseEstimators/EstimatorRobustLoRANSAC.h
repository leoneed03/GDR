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

        bool printProgressToCout = true;

        InlierCounter inlierCounter;
        ParamsRANSAC paramsLoRansac;

        SE3 optimizeOnInliers(
                const EstimatorNPoints &estimatorNp,
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntrToBeTransformed,
                const CameraRGBD &cameraIntrDestination,
                const std::vector<std::pair<double, int>> &errorsAndInlierIndices,
                std::vector<std::pair<double, int>> &errorsAndInlierIndicesLocallyOptimized) const;

    public:
        EstimatorRobustLoRANSAC(const InlierCounter &inlierCounterToSet,
                                const ParamsRANSAC &paramsRansacToSet);

        bool getPrintProgressToCout() const;

        void setPrintProgressToCout(bool printProgress);

        SE3 estimateRelativePose(
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntrToBeTransformed,
                const CameraRGBD &cameraIntrDestination,
                bool &estimationSuccess,
                std::vector<int> &inlierIndices) override;

        virtual SE3
        getTransformationMatrixUmeyamaLoRANSAC(
                const Estimator3Points &estimator3p,
                const EstimatorNPoints &estimatorNp,
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntrToBeTransformed,
                const CameraRGBD &cameraIntrDestination,
                bool &estimationSuccess,
                std::vector<int> &inlierIndices) const;
    };
}

#endif
