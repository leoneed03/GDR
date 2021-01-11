//
// Created by leoneed on 12/22/20.
//

#include <algorithm>
#include <random>

#include "umeyama.h"
#include "printer.h"

namespace gdr {

    std::vector<std::pair<double, int>> getPartionedByNthElement(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                                 const Eigen::Matrix4Xd &destinationPoints,
                                                                 const Eigen::Matrix4d &cR_t_umeyama,
                                                                 int numberOfSeparatorElement) {

        int numInliers = numberOfSeparatorElement + 1;

        Eigen::Matrix4Xd pointsAfterTransformation = cR_t_umeyama * toBeTransormedPoints;

        std::vector<std::pair<double, int>> euclideanErrors(toBeTransormedPoints.cols(), {std::numeric_limits<double>::max(), -1});

        for (int pointCounter = 0; pointCounter < euclideanErrors.size(); ++pointCounter) {
            double currentError = (pointsAfterTransformation.col(pointCounter) -
                                   destinationPoints.col(pointCounter)).norm();
            euclideanErrors[pointCounter] = {
                    currentError,
                    pointCounter};
        }

        int lastInlierPos = std::max(numInliers - 1, 0);
        std::nth_element(euclideanErrors.begin(), euclideanErrors.begin() + lastInlierPos, euclideanErrors.end());

        return euclideanErrors;
    }



    Eigen::Matrix4d getTransformationMatrixUmeyamaLoRANSAC(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                           const Eigen::Matrix4Xd &destinationPoints,
                                                           int numIterationsRansac,
                                                           int numOfPoints,
                                                           double inlierCoeff,
                                                           bool &estimationSuccess,
                                                           double maxErrorCorrespondence) {

        estimationSuccess = true;
        int dim = 3;
        if (inlierCoeff > 1) {
            inlierCoeff = 1;
        }

        if (inlierCoeff < 0) {
            inlierCoeff = 0;
        }
        assert(numOfPoints == toBeTransormedPoints.cols());
        assert(toBeTransormedPoints.cols() == destinationPoints.cols());

        int numInliers = (int) (inlierCoeff * numOfPoints);
        int lastInlierPos = std::max(numInliers - 1, 0);

        Eigen::Matrix4d optimal_cR_t_umeyama_transformation;
        optimal_cR_t_umeyama_transformation.setIdentity();

        double minError = std::numeric_limits<double>::max();
        int attempt = -1;
        double mError = -1;
        std::vector<int> inlierIndices;


        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());
        std::uniform_int_distribution<> distrib(0, numOfPoints - 1);


        for (int i = 0; i < numIterationsRansac; ++i) {
            std::vector<int> p(dim, 0);
            Eigen::Matrix4Xd toBeTransformed3Points = Eigen::Matrix4Xd(dim + 1, dim);
            Eigen::Matrix4Xd dest3Points = Eigen::Matrix4Xd(dim + 1, dim);
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
                toBeTransformed3Points.col(j) = toBeTransormedPoints.col(p[j]);
                dest3Points.col(j) = destinationPoints.col(p[j]);
            }

            Eigen::Matrix4d cR_t_umeyama_3_points = umeyama(toBeTransformed3Points.block(0, 0, dim, dim),
                                                            dest3Points.block(0, 0, dim, dim));

            std::vector<std::pair<double, int>> euclideanErrors = getPartionedByNthElement(toBeTransormedPoints,
                                                                                           destinationPoints,
                                                                                           cR_t_umeyama_3_points,
                                                                                           lastInlierPos);
            std::pair<double, int> errorAndLastInlierNumber = euclideanErrors[lastInlierPos];
            Eigen::Matrix4Xd toBeTransformedInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);
            Eigen::Matrix4Xd destInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);
            for (int currentIndex = 0; currentIndex < numInliers; ++currentIndex) {
                int index = euclideanErrors[currentIndex].second;
                toBeTransformedInlierPoints.col(currentIndex) = toBeTransormedPoints.col(index);
                destInlierPoints.col(currentIndex) = destinationPoints.col(index);
            }

            double normError = errorAndLastInlierNumber.first;

            if (normError < minError) {
                PRINT_PROGRESS("att " << i << " with error " << normError
                                      << "++++++++++++++++++++++++++++++++++++++\n" << " total inliers " << numInliers);
                mError = normError;
                optimal_cR_t_umeyama_transformation = umeyama(
                        toBeTransformedInlierPoints.block(0, 0, dim, numInliers),
                        destInlierPoints.block(0, 0, dim, numInliers));
                attempt = i;
                minError = normError;
            }
        }
        PRINT_PROGRESS("cand \n" << optimal_cR_t_umeyama_transformation << "RANSAC found on attempt " << attempt
                                 << " error on last \'inlier\' " << mError);

        std::vector<std::pair<double, int>> totalEuclideanErrors = getPartionedByNthElement(toBeTransormedPoints,
                                                                                            destinationPoints,
                                                                                            optimal_cR_t_umeyama_transformation,
                                                                                            lastInlierPos);
        auto errorAndLastInlierNumber = totalEuclideanErrors[lastInlierPos];

        estimationSuccess = errorAndLastInlierNumber.first < maxErrorCorrespondence;

        return optimal_cR_t_umeyama_transformation;
    }

}


