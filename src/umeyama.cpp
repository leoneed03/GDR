//
// Created by leoneed on 12/22/20.
//

#define DEBUG_PRINT_UMEYAMA 0

#include <iomanip>
#include <iostream>

#include "umeyama.h"

Eigen::Matrix4d getTransformationMatrixUmeyamaLoRANSAC(const MatrixX &toBeTransormedPoints,
                                                       const MatrixX &destinationPoints,
                                                       int numIterationsRansac,
                                                       int numOfPoints,
                                                       double inlierCoeff) {
    int dim = 3;
    int top = 10;
    if (inlierCoeff > 1) {
        inlierCoeff = 1;
    }

    if (inlierCoeff < 0) {
        inlierCoeff = 0;
    }
    assert(numOfPoints == toBeTransormedPoints.cols());
    assert(toBeTransormedPoints.cols() == destinationPoints.cols());

    int numInliers = (int) (inlierCoeff * numOfPoints);

    std::vector<int> pointsPositions;
    pointsPositions.reserve(numOfPoints);
    for (int i = 0; i < numOfPoints; ++i) {
        pointsPositions.push_back(i);
    }
    Eigen::Matrix4d bestMath;

    double minError = 1e6;
    int attempt = -1;
    double mError = -1;
    std::vector<int> inlierIndices;
    srand((unsigned int) time(0));

    Eigen::Matrix4d cR_t_umeyama_3_points_cand;
    std::vector<int> triple;

    for (int i = 0; i < numIterationsRansac; ++i) {
        std::vector<int> p(dim, 0);
        MatrixX toBeTransformed3Points = MatrixX(dim + 1, dim);
        MatrixX dest3Points = MatrixX(dim + 1, dim);
        p[0] = rand() % numOfPoints;
        p[1] = rand() % numOfPoints;
        p[2] = rand() % numOfPoints;

        while (p[0] == p[1]) {
            p[1] = rand() % numOfPoints;
        }
        while (p[0] == p[2] || p[1] == p[2]) {
            p[2] = rand() % numOfPoints;
        }
        for (int j = 0; j < p.size(); ++j) {
            toBeTransformed3Points.col(j) = toBeTransormedPoints.col(p[j]);
            dest3Points.col(j) = destinationPoints.col(p[j]);
            for (int assertCounter = 0; assertCounter < dim; ++assertCounter) {
                assert(toBeTransormedPoints.col(p[j])[assertCounter] == toBeTransformed3Points.col(j)[assertCounter]);
                assert(destinationPoints.col(p[j])[assertCounter] == dest3Points.col(j)[assertCounter]);
            }
        }

        Eigen::Matrix4d cR_t_umeyama_3_points = umeyama(toBeTransformed3Points.block(0, 0, dim, dim),
                                                        dest3Points.block(0, 0, dim, dim));
        std::sort(pointsPositions.begin(), pointsPositions.end(),
                  [toBeTransormedPoints, destinationPoints, dim, cR_t_umeyama_3_points](const auto &lhs,
                                                                                        const auto &rhs) {
                      auto &toBeTransformedLeft = toBeTransormedPoints.col(lhs);
                      auto &toBeTransformedRight = toBeTransormedPoints.col(rhs);
                      auto &destinationLeft = destinationPoints.col(lhs);
                      auto &destinationRight = destinationPoints.col(rhs);
                      double dist1 = 0;
                      double dist2 = 0;
                      auto &destLeft = cR_t_umeyama_3_points * toBeTransformedLeft;
                      auto &destRight = cR_t_umeyama_3_points * toBeTransformedRight;
                      for (int pp = 0; pp < dim; ++pp) {
                          dist1 += pow(destLeft[pp] - destinationLeft[pp], 2);
                          dist2 += pow(destRight[pp] - destinationRight[pp], 2);
                      }
                      return dist1 < dist2;
                  });

        MatrixX toBeTransformedInlierPoints = MatrixX(dim + 1, numInliers);
        MatrixX destInlierPoints = MatrixX(dim + 1, numInliers);
        for (int currentIndex = 0; currentIndex < numInliers; ++currentIndex) {
            int index = pointsPositions[currentIndex];
            toBeTransformedInlierPoints.col(currentIndex) = toBeTransormedPoints.col(index);
            destInlierPoints.col(currentIndex) = destinationPoints.col(index);

            assert(toBeTransformedInlierPoints.col(currentIndex)[1] == toBeTransormedPoints.col(index)[1]);
            assert(destInlierPoints.col(currentIndex)[2] == destinationPoints.col(index)[2]);
        }

        const auto &toBeTransformedColumn = toBeTransformedInlierPoints.col(std::max(numInliers - 1, 0));
        const auto &destColumn = destInlierPoints.col(std::max(numInliers - 1, 0));
        auto dest = cR_t_umeyama_3_points * toBeTransformedColumn;
        double normError = 0;
        for (int pp = 0; pp < dim; ++pp) {
            normError += pow(dest[pp] - destColumn[pp], 2);
        }
        Eigen::Matrix4d cR_t_umeyama_inlier_points = cR_t_umeyama_3_points;
        if (normError < minError) {
            cR_t_umeyama_inlier_points = umeyama(toBeTransformedInlierPoints.block(0, 0, dim, numInliers),
                                                 destInlierPoints.block(0, 0, dim, numInliers));
        }

        if (DEBUG_PRINT_UMEYAMA) {
            std::cout << "att " << std::setw(6) << i << " with error " << normError << std::endl;

            std::cout << "++++++++++++++++++++++++++++++++++++++\n" << " total inliers " << numInliers << std::endl;
        }
        if (normError < minError) {
            cR_t_umeyama_3_points_cand = cR_t_umeyama_3_points;
            mError = normError;
            bestMath = cR_t_umeyama_inlier_points;
            attempt = i;
            inlierIndices = pointsPositions;
            minError = normError;
            triple = p;
        }
    }
    if (DEBUG_PRINT_UMEYAMA) {
        std::cout << "cand \n" << cR_t_umeyama_3_points_cand << std::endl;
        std::cout << "RANSAC found on attempt " << attempt << " error on last \'inlier\' " << mError << std::endl;
        for (int i = 0; i < top; ++i) {
            std::cout << std::setw(6) << inlierIndices[i];
        }
        std::cout << std::endl;
        for (int i = 0; i < triple.size(); ++i) {
            std::cout << std::setw(6) << triple[i];
        }
        std::cout << std::endl;
    }
    return bestMath;
}
