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

    std::ostream &operator<<(const std::ostream &os, const ErrorInformation &info) {

        std::cout << info.nameError << ".rmse   " << info.RMSE << " " << info.typeOfMeasurement << std::endl;
        std::cout << info.nameError << ".mean   " << info.MEAN << " " << info.typeOfMeasurement  << std::endl;
        std::cout << info.nameError << ".median " << info.MEDIAN << " " << info.typeOfMeasurement  << std::endl;
        std::cout << info.nameError << ".std    " << info.STD << " " << info.typeOfMeasurement  << std::endl;
        std::cout << info.nameError << ".min    " << info.MINERR << " " << info.typeOfMeasurement  << std::endl;
        std::cout << info.nameError << ".max    " << info.MAXERR << " " << info.typeOfMeasurement  << std::endl;
    }

    struct ErrorRotationTranslation {
        int numberOfPosesEvaluated = 0;
        int numberOfPosesTrajectory = 0;
        int numberOfPosesGroundTruth = 0;
        ErrorInformation rotationError;
        ErrorInformation translationError;

        ErrorRotationTranslation(const ErrorInformation &rotationErrorToSet,
                                 const ErrorInformation &translationErrorToSet) :
                rotationError(rotationErrorToSet),
                translationError(translationErrorToSet) {};
    };

    class Evaluator {

        std::vector<PoseFullInfo> posesGroundTruth;
        std::map<double, PoseFullInfo> poseGroundTruthByTime;

        void initFromPosesVector(const std::vector<PoseFullInfo> &poses);

        static double median(const std::vector<double> &values);

        static void sort(std::vector<double> &values);

    public:
        Evaluator(const std::string &groundtruthFile);

        Evaluator(const std::vector<PoseFullInfo> &poses);

        ErrorRotationTranslation evaluateTrajectory(const std::vector<PoseFullInfo> &trajectory,
                                                    int indexFixed = 0,
                                                    double maxTimeDiff = 0.02) const;
    };
}


#endif
