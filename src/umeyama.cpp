//
// Created by leoneed on 12/22/20.
//

#include <algorithm>
#include <random>
#include <thread>

#include "umeyama.h"
#include "printer.h"
#include <mutex>
#include <tbb/parallel_for.h>

namespace gdr {


    // return vector of pairs for each inlier: {errorInPixelsAfterTransformationAndProjection, columnNumber in Original Matrix toBeTransformedpoints}
    std::vector<std::pair<double, int>> calculateProjectionErrors(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                                  const Eigen::Matrix4Xd &destinationPoints,
                                                                  const Eigen::Matrix3d &cameraIntr3x3Destination,
                                                                  const Eigen::Matrix4d &cR_t_umeyama,
                                                                  double maxProjectionErrorPixels) {

        assert(toBeTransormedPoints.cols() == destinationPoints.cols());
        Eigen::Matrix4Xd pointsAfterTransformation = cR_t_umeyama * toBeTransormedPoints;

        std::vector<std::pair<double, int>> projectionErrorsInliers;
        assert(toBeTransormedPoints.cols() == pointsAfterTransformation.cols());

        for (int pointCounter = 0; pointCounter < pointsAfterTransformation.cols(); ++pointCounter) {
//            std::cout << cameraIntr3x3Destination << std::endl;
            Eigen::Vector3d toBeTransformedProjection =
                    cameraIntr3x3Destination * pointsAfterTransformation.col(pointCounter).topLeftCorner<3, 1>();
            Eigen::Vector3d destinationPointProjection =
                    cameraIntr3x3Destination * destinationPoints.col(pointCounter).topLeftCorner<3, 1>();
            for (int i = 0; i < 2; ++i) {
                toBeTransformedProjection[i] /= toBeTransformedProjection[2];
                destinationPointProjection[i] /= destinationPointProjection[2];
            }

            // depth error can be also checked
            auto projectionErrorPixels2d = (toBeTransformedProjection.topLeftCorner<2, 1>() -
                                            destinationPointProjection.topLeftCorner<2, 1>());

            double maxError = std::max(std::abs(projectionErrorPixels2d[0]), std::abs(projectionErrorPixels2d[1]));
            if (maxError < maxProjectionErrorPixels) {
                projectionErrorsInliers.push_back({maxError, pointCounter});
            }
        }

        return projectionErrorsInliers;
    }


    std::vector<std::pair<double, int>> getPartionedByNthElement(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                                 const Eigen::Matrix4Xd &destinationPoints,
                                                                 const Eigen::Matrix4d &cR_t_umeyama,
                                                                 int numberOfSeparatorElement) {

        int numInliers = numberOfSeparatorElement + 1;

        Eigen::Matrix4Xd pointsAfterTransformation = cR_t_umeyama * toBeTransormedPoints;

        std::vector<std::pair<double, int>> euclideanErrors(toBeTransormedPoints.cols(),
                                                            {std::numeric_limits<double>::max(), -1});

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

    Eigen::Matrix4d getTransformationMatrixUmeyamaLoRANSACProjectiveError(const Eigen::Matrix4Xd &toBeTransormedPoints,
                                                                          const Eigen::Matrix4Xd &destinationPoints,
                                                                          const Eigen::Matrix3d &cameraIntr3x3ToBeTransformed,
                                                                          const Eigen::Matrix3d &cameraIntr3x3Destination,
                                                                          const int numIterationsRansac,
                                                                          const int numOfPoints,
                                                                          double inlierCoeff,
                                                                          bool &estimationSuccess,
                                                                          double maxProjectionErrorPixels) {

        estimationSuccess = true;
        int dim = 3;
        if (inlierCoeff > 1) {
            inlierCoeff = 1;
        }

        if (inlierCoeff < 0) {
            inlierCoeff = 0;
        }

        int totalNumberInliers = 0;
        assert(numOfPoints == toBeTransormedPoints.cols());
        assert(toBeTransormedPoints.cols() == destinationPoints.cols());

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
                toBeTransformed3Points.col(j) = toBeTransormedPoints.col(p[j]);
                dest3Points.col(j) = destinationPoints.col(p[j]);
            }

            Eigen::Matrix4d cR_t_umeyama_3_points = umeyama(toBeTransformed3Points.block(0, 0, dim, dim),
                                                            dest3Points.block(0, 0, dim, dim));

            /*
            std::cout << dest3Points << std::endl;
            std::cout << "toBeTransformed" << std::endl;
            std::cout << toBeTransformed3Points << std::endl;
            std::cout << "________________________" << std::endl;
            std::cout << "after cR_t * toBeTransformed" << std::endl;
            std::cout << cR_t_umeyama_3_points * toBeTransformed3Points << std::endl;
            std::cout << "after cR_t * dest" << std::endl;
            std::cout << cR_t_umeyama_3_points * dest3Points << std::endl;
            assert((dest3Points - cR_t_umeyama_3_points * toBeTransformed3Points).norm() < 3 * std::numeric_limits<double>::epsilon());
            */
            std::vector<std::pair<double, int>> projectionErrorsAndInlierIndices = calculateProjectionErrors(
                    toBeTransormedPoints,
                    destinationPoints,
                    cameraIntr3x3Destination,
                    cR_t_umeyama_3_points,
                    maxProjectionErrorPixels);

            int numInliers = projectionErrorsAndInlierIndices.size();

            Eigen::Matrix4Xd toBeTransformedInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);
            Eigen::Matrix4Xd destInlierPoints = Eigen::Matrix4Xd(dim + 1, numInliers);
            for (int currentIndex = 0; currentIndex < numInliers; ++currentIndex) {
                int index = projectionErrorsAndInlierIndices[currentIndex].second;
                toBeTransformedInlierPoints.col(currentIndex) = toBeTransormedPoints.col(index);
                destInlierPoints.col(currentIndex) = destinationPoints.col(index);
            }

            if (numInliers > totalNumberInliers) {
                totalNumberInliers = numInliers;
                optimal_cR_t_umeyama_transformation = umeyama(
                        toBeTransformedInlierPoints.block(0, 0, dim, numInliers),
                        destInlierPoints.block(0, 0, dim, numInliers));
                attempt = i;

            }
        }


        std::vector<std::pair<double, int>> totalProjectionErrorsAndInlierIndices = calculateProjectionErrors(
                toBeTransormedPoints,
                destinationPoints,
                cameraIntr3x3Destination,
                optimal_cR_t_umeyama_transformation,
                maxProjectionErrorPixels);

        int numberInliersAfterLocalOptimization = totalProjectionErrorsAndInlierIndices.size();
        estimationSuccess = numberInliersAfterLocalOptimization > inlierCoeff * toBeTransormedPoints.cols();

        std::string logs;
        std::cout << "before LO " << totalNumberInliers << std::endl;
        if (estimationSuccess) {
            std::cout << "success, inliers " << numberInliersAfterLocalOptimization << " of "
                      << toBeTransormedPoints.cols() << " ratio "
                      << (double) numberInliersAfterLocalOptimization / toBeTransormedPoints.cols();
        } else {

            std::cout << "not success, inliers ONLY " << numberInliersAfterLocalOptimization << " of "
                      << toBeTransormedPoints.cols() << " ratio "
                      << (double) numberInliersAfterLocalOptimization / toBeTransormedPoints.cols();
        }
        std::cout << std::endl;
        return optimal_cR_t_umeyama_transformation;
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
        double mError = -1;
        std::vector<int> inlierIndices;

        std::mutex compareWithOptimal;

        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());
        std::uniform_int_distribution<> distrib(0, numOfPoints - 1);


        tbb::parallel_for(0, numIterationsRansac, [dim, &distrib, &randomNumberGenerator, &toBeTransormedPoints,
                &destinationPoints, lastInlierPos, numInliers, &compareWithOptimal,
                &minError, &mError, &optimal_cR_t_umeyama_transformation](int) {
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

            {
                std::unique_lock<std::mutex> lockCompareWithOptimal(compareWithOptimal);
                if (normError < minError) {
                    std::cout << "umeyama thread received better results " << std::this_thread::get_id() << std::endl;
                    mError = normError;
                    optimal_cR_t_umeyama_transformation = umeyama(
                            toBeTransformedInlierPoints.block(0, 0, dim, numInliers),
                            destInlierPoints.block(0, 0, dim, numInliers));
                    minError = normError;
                } else {

//                    std::cout << "umeyama thread NOT received better results " << std::this_thread::get_id() << std::endl;
                }
            }
        });

        std::vector<std::pair<double, int>> totalEuclideanErrors = getPartionedByNthElement(toBeTransormedPoints,
                                                                                            destinationPoints,
                                                                                            optimal_cR_t_umeyama_transformation,
                                                                                            lastInlierPos);
        auto errorAndLastInlierNumber = totalEuclideanErrors[lastInlierPos];

        estimationSuccess = errorAndLastInlierNumber.first < maxErrorCorrespondence;

        return optimal_cR_t_umeyama_transformation;
    }

}


