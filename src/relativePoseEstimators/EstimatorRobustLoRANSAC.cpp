//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <relativePoseEstimators/EstimatorRobustLoRANSAC.h>

namespace gdr {

    SE3 EstimatorRobustLoRANSAC::getTransformationMatrixUmeyamaLoRANSACProjectiveError(
            const Estimator3Points &estimator3p,
            const EstimatorNPoints &estimatorNp,
            const InlierCounter &inlierCounter,
            const Eigen::Matrix4Xd &toBeTransformedPoints,
            const Eigen::Matrix4Xd &destinationPoints,
            const CameraRGBD &cameraIntrToBeTransformed,
            const CameraRGBD &cameraIntrDestination,
            const ParamsRANSAC &paramsRansac,
            bool &estimationSuccess,
            std::vector<int> &inlierIndicesToReturn) const {

        int numIterationsRansac = paramsRansac.getNumIterations();
        double inlierCoeff = paramsRansac.getInlierCoeff();
        double max3DError = paramsRansac.getMax3DError();
        double maxProjectionErrorPixels = paramsRansac.getMaxProjectionErrorPixels();
        bool useProjection = paramsRansac.getProjectionUsage();

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

        SE3 optimal_cR_t_umeyama_transformation;

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
                            paramsRansac);

            int numInliers = projectionErrorsAndInlierIndices.size();

            if (numInliers > totalNumberInliers && numInliers >= minPointNumberEstimator) {

                if (getPrintProgressToCout()) {
                    std::cout << "3 point NEW estimator got better results: " << numInliers
                              << " vs old " << totalNumberInliers << std::endl;
                }

                optimal_cR_t_umeyama_transformation = cR_t_umeyama_3_points;
                totalNumberInliers = numInliers;
                Eigen::Matrix4Xd toBeTransformedInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);
                Eigen::Matrix4Xd destInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);

                for (int currentIndex = 0; currentIndex < numInliers; ++currentIndex) {
                    int index = projectionErrorsAndInlierIndices[currentIndex].second;
                    toBeTransformedInlierPoints.col(currentIndex) = toBeTransformedPoints.col(index);
                    destInlierPoints.col(currentIndex) = destinationPoints.col(index);
                }
                auto inlier_optimal_cR_t_umeyama_transformation = estimatorNp.getRt(
                        toBeTransformedInlierPoints,
                        destInlierPoints,
                        cameraIntrToBeTransformed,
                        cameraIntrDestination);

                std::vector<std::pair<double, int>> projectionErrorsAndInlierIndicesAfterLocalOptimization =
                        inlierCounter.calculateInlierProjectionErrors(
                                toBeTransformedPoints,
                                destinationPoints,
                                cameraIntrDestination,
                                inlier_optimal_cR_t_umeyama_transformation,
                                paramsRansac);

                if (getPrintProgressToCout()) {
                    std::cout << "         NEW after LO number Of Inliers is "
                              << projectionErrorsAndInlierIndicesAfterLocalOptimization.size()
                              << " vs " << totalNumberInliers << std::endl;
                }

                if (projectionErrorsAndInlierIndicesAfterLocalOptimization.size() >= totalNumberInliers) {
                    optimal_cR_t_umeyama_transformation = inlier_optimal_cR_t_umeyama_transformation;
                    totalNumberInliers = projectionErrorsAndInlierIndicesAfterLocalOptimization.size();
                }
            }
        }


        std::vector<std::pair<double, int>> totalProjectionErrorsAndInlierIndices =
                inlierCounter.calculateInlierProjectionErrors(
                        toBeTransformedPoints,
                        destinationPoints,
                        cameraIntrDestination,
                        optimal_cR_t_umeyama_transformation,
                        paramsRansac);

        int numberInliersOptimal = totalProjectionErrorsAndInlierIndices.size();
        estimationSuccess = numberInliersOptimal > inlierCoeff * toBeTransformedPoints.cols();

        std::string logs;

        std::vector<int> inlierIndices;
        inlierIndices.reserve(totalProjectionErrorsAndInlierIndices.size());

        for (const auto &inlierErrorAndIndex: totalProjectionErrorsAndInlierIndices) {
            inlierIndices.emplace_back(inlierErrorAndIndex.second);
        }

        std::swap(inlierIndices, inlierIndicesToReturn);
        return optimal_cR_t_umeyama_transformation;
    }

    SE3 EstimatorRobustLoRANSAC::estimateRelativePose(const Eigen::Matrix4Xd &toBeTransformedPoints,
                                                      const Eigen::Matrix4Xd &destinationPoints,
                                                      const CameraRGBD &cameraIntrToBeTransformed,
                                                      const CameraRGBD &cameraIntrDestination,
                                                      const ParamsRANSAC &paramsRansac,
                                                      bool &estimationSuccess,
                                                      std::vector<int> &inlierIndices) {
        return getTransformationMatrixUmeyamaLoRANSACProjectiveError(
                Estimator3Points(),
                EstimatorNPoints(),
                InlierCounter(),
                toBeTransformedPoints,
                destinationPoints,
                cameraIntrToBeTransformed,
                cameraIntrDestination,
                ParamsRANSAC(),
                estimationSuccess,
                inlierIndices);
    }

    bool EstimatorRobustLoRANSAC::getPrintProgressToCout() const {
        return printProgressToCout;
    }

    void EstimatorRobustLoRANSAC::setPrintProgressToCout(bool printProgress) {
        printProgressToCout = printProgress;
    }
}