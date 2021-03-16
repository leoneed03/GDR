//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_PARAMSRANSAC_H
#define GDR_PARAMSRANSAC_H

namespace gdr {

    // locally optimized ransac parameters
    struct ParamsRANSAC {
    private:
        // min proportion of inliers between matches
        double minInlierCoefficient = 0.5;
        // min number of inliers between matches
        int minInliersNumber = 10;

        int numIterations = 100;

        // max L_{pMetric} error between keypoints on the image to be counted as an inlier
        double maxProjectionErrorPixels = 2.0;
        int pMetric = 1;
        // max L2 error in meters to be counted as an inlier
        double max3DError = 0.05;

        // which error model to choose
        bool useProjectionError = true;

        // max number of threads to use (automatic detection by default)
        int maxNumberOfThreads = -1;
    public:

        double getInlierCoeff() const;

        void setInlierCoeff(double minInlierCoeff);

        int getInlierNumber() const;

        void setInlierNumber(int inlierNumber);

        int getNumIterations() const;

        void setNumIterations(int numIterations);

        double getMaxProjectionErrorPixels() const;

        void setMaxProjectionErrorPixels(double maxProjectionErrorPixels);

        int getLpMetricParam() const;

        void setLpMetricParam(int p);

        double getMax3DError() const;

        void setMax3DError(double max3DError);

        bool getProjectionUsage() const;

        void setProjectionUsage(bool useProjection);

        int getMaxNumberOfThreads() const;

        void setMaxNumberOfThreads(int maxNumberOfThreads);
    };
}

#endif
