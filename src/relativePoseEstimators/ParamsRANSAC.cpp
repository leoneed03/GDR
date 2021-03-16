//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseEstimators/ParamsRANSAC.h"

namespace gdr {

    double ParamsRANSAC::getInlierCoeff() const {
        return minInlierCoefficient;
    }

    void ParamsRANSAC::setInlierCoeff(double newMinInlierCoeff) {
        minInlierCoefficient = newMinInlierCoeff;
    }

    int ParamsRANSAC::getInlierNumber() const {
        return minInliersNumber;
    }

    void ParamsRANSAC::setInlierNumber(int inlierNumber) {
        minInliersNumber = inlierNumber;
    }

    int ParamsRANSAC::getNumIterations() const {
        return numIterations;
    }

    void ParamsRANSAC::setNumIterations(int newNumIterations) {
        numIterations = newNumIterations;
    }

    double ParamsRANSAC::getMaxProjectionErrorPixels() const {
        return maxProjectionErrorPixels;
    }

    void ParamsRANSAC::setMaxProjectionErrorPixels(double newMaxProjectionErrorPixels) {
        maxProjectionErrorPixels = newMaxProjectionErrorPixels;
    }

    int ParamsRANSAC::getLpMetricParam() const {
        return pMetric;
    }

    void ParamsRANSAC::setLpMetricParam(int p) {
        pMetric = p;
    }

    double ParamsRANSAC::getMax3DError() const {
        return max3DError;
    }

    void ParamsRANSAC::setMax3DError(double newMax3DError) {
        max3DError = newMax3DError;
    }

    bool ParamsRANSAC::getProjectionUsage() const {
        return useProjectionError;
    }

    void ParamsRANSAC::setProjectionUsage(bool useProjection) {
        useProjectionError = useProjection;
    }

    int ParamsRANSAC::getMaxNumberOfThreads() const {
        return maxNumberOfThreads;
    }

    void ParamsRANSAC::setMaxNumberOfThreads(int newNumberOfThreads) {
        maxNumberOfThreads = newNumberOfThreads;
    }
}

