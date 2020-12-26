//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "poseEstimation.h"
#include "groundTruthTransformer.h"

#include <unordered_map>
#include <Eigen/Eigen>
#include <cassert>
#include <cmath>

gdr::errorStats::errorStats(double mse, double msd) : meanError(mse), standartDeviation(msd) {}

std::pair<gdr::errorStats, gdr::errorStats> gdr::getErrorStatsTranslationRotation(
        const std::vector<std::map<int, std::pair<double, double>>> &errorRelativePosesTranslationRotation) {

    double sumT = 0;
    double sumR = 0;
    double sumTsquared = 0;
    double sumRsquared = 0;
    int numberMeasurements = 0;

    for (int i = 0; i < errorRelativePosesTranslationRotation.size(); ++i) {
        for (const auto &errorTranslationRotationEntry: errorRelativePosesTranslationRotation[i]) {
            const auto &errorTR = errorTranslationRotationEntry.second;
            ++numberMeasurements;
            sumT += errorTR.first;
            sumR += errorTR.second;
            sumTsquared += pow(errorTR.first, 2);
            sumRsquared += pow(errorTR.second, 2);
        }
    }
    double meanT = sumT / numberMeasurements;
    double meanR = sumR / numberMeasurements;
    double meanTsquared = sumTsquared / numberMeasurements;
    double meanRsquared = sumRsquared / numberMeasurements;
    double standartDeviationR = meanRsquared - pow(meanR, 2);
    double standartDeviationT = meanTsquared - pow(meanT, 2);

    errorStats errorT(meanT, standartDeviationT);
    errorStats errorR(meanR, standartDeviationR);

    return {errorT, errorR};
}


std::pair<gdr::errorStats, gdr::errorStats> gdr::getErrorStatsTranslationRotationFromGroundTruthAndEstimatedPairWise(
        const std::string &pathToGroundTruth, const std::string &estimatedPairWise) {

    std::string groundTruthPairWiseTransformations = "pairwiseTransformations.txt";

    gdr::GTT::extractAllRelativeTransformationPairwise(pathToGroundTruth, groundTruthPairWiseTransformations);

    std::vector<std::vector<std::vector<double>>> groundTruthMeasurements = gdr::getMeasurements(
            groundTruthPairWiseTransformations);
    std::vector<std::vector<std::vector<double>>> estimatedMeasurements = gdr::getMeasurements(estimatedPairWise);

    std::vector<std::map<int, std::pair<double, double>>> tableErrorsTranslationRotation = gdr::getErrorsTranslationRotation(
            groundTruthMeasurements, estimatedMeasurements);
    std::pair<gdr::errorStats, gdr::errorStats> errorsStatsTR = gdr::getErrorStatsTranslationRotation(
            tableErrorsTranslationRotation);

    return errorsStatsTR;
}


std::vector<std::map<int, std::pair<double, double>>>
gdr::getErrorsTranslationRotation(const std::vector<std::vector<std::vector<double>>> &truePairWiseTransformations,
                                  const std::vector<std::vector<std::vector<double>>> &estimatedPairWiseTransformations) {

    int poseNumber = estimatedPairWiseTransformations.size();
    assert(estimatedPairWiseTransformations.size() == truePairWiseTransformations.size());

    std::vector<std::map<int, std::pair<double, double>>>
            tableOfErrorsTranslationRotation(poseNumber);

    for (int i = 0; i < estimatedPairWiseTransformations.size(); ++i) {
        for (int j = 0; j < estimatedPairWiseTransformations[i].size(); ++j) {
            const auto &pose = estimatedPairWiseTransformations[i][j];
            if (pose.empty()) {
                continue;
            }
            assert(pose.size() == 7);
            std::vector<double> estimatedT = {pose[0], pose[1], pose[2]};
            std::vector<double> estimatedQuatVector = {pose[3], pose[4], pose[5], pose[6]};

            Eigen::Quaterniond estimatedQuaternion(estimatedQuatVector.data());
            Eigen::Vector3d estimatedTranslation(estimatedT.data());

            const auto &truePose = truePairWiseTransformations[i][j];
            assert(truePose.size() == 7);
            std::vector<double> trueT = {truePose[0], truePose[1], truePose[2]};
            std::vector<double> trueQuatVector = {truePose[3], truePose[4], truePose[5], truePose[6]};

            Eigen::Quaterniond trueQuaternion(trueQuatVector.data());
            Eigen::Vector3d trueTranslation(trueT.data());

            double errorRotation = abs(1 - (trueQuaternion.inverse() * estimatedQuaternion).norm());
            double errorTranslation = (trueTranslation - estimatedTranslation).norm();

            tableOfErrorsTranslationRotation[i][j] = {errorTranslation, errorRotation};
        }
    }

    return tableOfErrorsTranslationRotation;
}