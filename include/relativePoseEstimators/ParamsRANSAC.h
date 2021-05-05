//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_PARAMSRANSAC_H
#define GDR_PARAMSRANSAC_H

namespace gdr {

    /** Locally optimized ransac parameters */
    struct ParamsRANSAC {

    public:
        /** min proportion of inliers between matches */
        double minInlierCoefficient = 0.6;

        /** min number of inliers between matches */
        int minInliersNumber = 15;

        int numIterations = 150;

        /** max L_{pMetric} error between keypoints on the image to be counted as an inlier */
        double maxProjectionErrorPixels = 2.0;
        int pMetric = 2;

        /** max L2 error in meters to be counted as an inlier */
        double maxL2ErrorMeters = 0.02;

        /** true if should use reprojection L_p error for inlier detection */
        bool useProjectionError = false;

        /** max number of threads to use */
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

        bool useProjection() const;

        bool useErrorL2() const;

        void setProjectionUsage(bool useProjection);

        int getMaxNumberOfThreads() const;

        void setMaxNumberOfThreads(int maxNumberOfThreads);

        double getAutoThreshold() const;
    };
}

#endif
