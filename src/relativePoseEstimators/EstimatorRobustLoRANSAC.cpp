//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <relativePoseEstimators/EstimatorRobustLoRANSAC.h>

namespace gdr {

    SE3 EstimatorRobustLoRANSAC::optimizeOnInliers(
            const EstimatorNPoints &estimatorNp,
            const Eigen::Matrix4Xd &toBeTransformedPoints,
            const Eigen::Matrix4Xd &destinationPoints,
            const CameraRGBD &cameraIntrToBeTransformed,
            const CameraRGBD &cameraIntrDestination,
            const std::vector<std::pair<double, int>> &errorsAndInlierIndices,
            std::vector<std::pair<double, int>> &errorsAndInliersIndicesLocallyOptimized) const {

        int dim = 3;
        int numInliers = errorsAndInlierIndices.size();

        Eigen::Matrix4Xd toBeTransformedInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);
        Eigen::Matrix4Xd destInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);

        for (int currentIndex = 0; currentIndex < numInliers; ++currentIndex) {
            int index = errorsAndInlierIndices[currentIndex].second;

            toBeTransformedInlierPoints.col(currentIndex) = toBeTransformedPoints.col(index);
            destInlierPoints.col(currentIndex) = destinationPoints.col(index);
        }
        auto inlier_optimal_cR_t_umeyama_transformation = estimatorNp.getRt(
                toBeTransformedInlierPoints,
                destInlierPoints,
                cameraIntrToBeTransformed,
                cameraIntrDestination);

        errorsAndInliersIndicesLocallyOptimized =
                inlierCounter.calculateInlierProjectionErrors(
                        toBeTransformedPoints,
                        destinationPoints,
                        cameraIntrDestination,
                        inlier_optimal_cR_t_umeyama_transformation,
                        paramsLoRansac);


        return inlier_optimal_cR_t_umeyama_transformation;
    }

    SE3 EstimatorRobustLoRANSAC::getTransformationMatrixUmeyamaLoRANSAC(
            const Estimator3Points &estimator3p,
            const EstimatorNPoints &estimatorNp,
            const Eigen::Matrix4Xd &toBeTransformedPoints,
            const Eigen::Matrix4Xd &destinationPoints,
            const CameraRGBD &cameraIntrToBeTransformed,
            const CameraRGBD &cameraIntrDestination,
            bool &estimationSuccess,
            std::vector<int> &inlierIndices) const {

        int numIterationsRansac = paramsLoRansac.getNumIterations();
        double inlierCoeff = paramsLoRansac.getInlierCoeff();

        estimationSuccess = true;
        int dim = 3;
        if (inlierCoeff > 1) {
            inlierCoeff = 1;
        }

        if (inlierCoeff < 0) {
            inlierCoeff = 0;
        }

        int totalNumberInliers = 0;
        int minPointNumberEstimator = 3;
        assert(toBeTransformedPoints.cols() >= minPointNumberEstimator);
        assert(toBeTransformedPoints.cols() == destinationPoints.cols());

        SE3 optimalSE3Transformation;

        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());
        int numOfPoints = toBeTransformedPoints.cols();
        std::uniform_int_distribution<> distrib(0, numOfPoints - 1);


        for (int i = 0; i < numIterationsRansac; ++i) {
            std::vector<int> p(dim, 0);
            Eigen::Matrix4Xd toBeTransformed3Points = Eigen::Matrix4Xd(dim + 1, dim);
            Eigen::Matrix4Xd dest3Points = Eigen::Matrix4Xd(dim + 1, dim);
            toBeTransformed3Points.setOnes();
            dest3Points.setOnes();
            p[0] = distrib(randomNumberGenerator);
            p[1] = distrib(randomNumberGenerator);
            p[2] = distrib(randomNumberGenerator);

            while (p[0] == p[1]) {
                p[1] = distrib(randomNumberGenerator);
            }
            while (p[0] == p[2] || p[1] == p[2]) {
                p[2] = distrib(randomNumberGenerator);
            }
            for (int j = 0; j < p.size(); ++j) {
                toBeTransformed3Points.col(j) = toBeTransformedPoints.col(p[j]);
                dest3Points.col(j) = destinationPoints.col(p[j]);
            }

            SE3 cR_t_umeyama_3_points = estimator3p.getRt(toBeTransformed3Points,
                                                          dest3Points,
                                                          cameraIntrToBeTransformed,
                                                          cameraIntrDestination);

            std::vector<std::pair<double, int>> projectionErrorsAndInlierIndices =
                    inlierCounter.calculateInlierProjectionErrors(
                            toBeTransformedPoints,
                            destinationPoints,
                            cameraIntrDestination,
                            cR_t_umeyama_3_points,
                            paramsLoRansac);

            int numInliers = projectionErrorsAndInlierIndices.size();

            if (numInliers > totalNumberInliers && numInliers >= minPointNumberEstimator) {

                optimalSE3Transformation = cR_t_umeyama_3_points;
                totalNumberInliers = numInliers;

                std::vector<std::pair<double, int>> errorsInliersLocOpt;

                auto locallyOptimizedRt =
                        optimizeOnInliers(estimatorNp,
                                          toBeTransformedPoints,
                                          destinationPoints,
                                          cameraIntrToBeTransformed,
                                          cameraIntrDestination,
                                          projectionErrorsAndInlierIndices,
                                          errorsInliersLocOpt);

                int numberInliersLocallyOptimized = errorsInliersLocOpt.size();

                if (numberInliersLocallyOptimized >= totalNumberInliers) {

                    optimalSE3Transformation = locallyOptimizedRt;
                    totalNumberInliers = numberInliersLocallyOptimized;

                    std::vector<std::pair<double, int>> errorsInliersLocOptTwice;


                    auto twiceLocallyOptimizedRt =
                            optimizeOnInliers(estimatorNp,
                                              toBeTransformedPoints,
                                              destinationPoints,
                                              cameraIntrToBeTransformed,
                                              cameraIntrDestination,
                                              errorsInliersLocOpt,
                                              errorsInliersLocOptTwice);

                    int numberInliersTwiceLocallyOptimizedTwice = errorsInliersLocOptTwice.size();

                    if (numberInliersTwiceLocallyOptimizedTwice > totalNumberInliers) {

                        optimalSE3Transformation = twiceLocallyOptimizedRt;
                        totalNumberInliers = numberInliersTwiceLocallyOptimizedTwice;

                    }
                }
            }
        }

        std::vector<std::pair<double, int>> totalProjectionErrorsAndInlierIndices =
                inlierCounter.calculateInlierProjectionErrors(
                        toBeTransformedPoints,
                        destinationPoints,
                        cameraIntrDestination,
                        optimalSE3Transformation,
                        paramsLoRansac);

        int numberInliersOptimal = totalProjectionErrorsAndInlierIndices.size();

        estimationSuccess = (numberInliersOptimal > inlierCoeff * toBeTransformedPoints.cols())
                            && (numberInliersOptimal >= paramsLoRansac.getInlierNumber());


        inlierIndices.clear();
        inlierIndices.reserve(totalProjectionErrorsAndInlierIndices.size());

        for (const auto &inlierErrorAndIndex: totalProjectionErrorsAndInlierIndices) {
            inlierIndices.emplace_back(inlierErrorAndIndex.second);
        }

        return optimalSE3Transformation;
    }

    SE3 EstimatorRobustLoRANSAC::estimateRelativePose(const Eigen::Matrix4Xd &toBeTransformedPoints,
                                                      const Eigen::Matrix4Xd &destinationPoints,
                                                      const CameraRGBD &cameraIntrToBeTransformed,
                                                      const CameraRGBD &cameraIntrDestination,
                                                      bool &estimationSuccess,
                                                      std::vector<int> &inlierIndices) {
        return getTransformationMatrixUmeyamaLoRANSAC(
                Estimator3Points(),
                EstimatorNPoints(),
                toBeTransformedPoints,
                destinationPoints,
                cameraIntrToBeTransformed,
                cameraIntrDestination,
                estimationSuccess,
                inlierIndices);
    }

    EstimatorRobustLoRANSAC::EstimatorRobustLoRANSAC(const InlierCounter &inlierCounterToSet,
                                                     const ParamsRANSAC &paramsRansacToSet) :
            inlierCounter(inlierCounterToSet),
            paramsLoRansac(paramsRansacToSet) {}
}