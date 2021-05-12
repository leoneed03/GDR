//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_EVALUATOR_H
#define GDR_EVALUATOR_H

#include <string>

#include "parametrization/PoseFullInfo.h"

namespace gdr {

    struct ErrorInformation {

        std::string nameError;
        std::string typeOfMeasurement;
        double RMSE, MEAN, MEDIAN, STD, MINERR, MAXERR;

        ErrorInformation() = default;

        ErrorInformation(const std::string &nameToSet, const std::string &typeToSet,
                         double rmse, double mean, double median, double std, double minErr, double maxErr) :
                nameError(nameToSet),
                typeOfMeasurement(typeToSet),
                RMSE(rmse),
                MEAN(mean),
                MEDIAN(median),
                STD(std),
                MINERR(minErr),
                MAXERR(maxErr) {}

        friend std::ostream &operator<<(const std::ostream &os, const ErrorInformation &info);
    };


    struct ErrorRotationTranslation {
        int numberOfPosesEvaluated = 0;
        int numberOfPosesTrajectory = 0;
        int numberOfPosesGroundTruth = 0;
        ErrorInformation rotationError;
        ErrorInformation translationError;

        ErrorRotationTranslation() = default;

        ErrorRotationTranslation(const ErrorInformation &rotationErrorToSet,
                                 const ErrorInformation &translationErrorToSet) :
                rotationError(rotationErrorToSet),
                translationError(translationErrorToSet) {};

        friend std::ostream &operator<<(std::ostream &os, const ErrorRotationTranslation &info);
    };


    class Evaluator {

        std::vector<PoseFullInfo> posesGroundTruth;
        std::map<double, PoseFullInfo> poseGroundTruthByTime;

        void initFromPosesVector(const std::vector<PoseFullInfo> &poses);

    public:
        static double median(const std::vector<double> &values);

        static void sort(std::vector<double> &values);

        Evaluator(const std::string &groundtruthFile);

        Evaluator(const std::vector<PoseFullInfo> &poses);

        ErrorRotationTranslation evaluateTrajectory(const std::vector<PoseFullInfo> &trajectory,
                                                    int indexFixed = 0,
                                                    bool alignWithUmeyama = false,
                                                    double maxTimeDiff = 0.02,
                                                    const std::string &pathOutAlignedGroundTruth = "",
                                                    const std::string &pathOutAlignedTrajectory = "") const;
    };
}


#endif
