//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IPOINTMATCHER_H
#define GDR_IPOINTMATCHER_H

namespace gdr {
    class IPointMatcher {

    public:
//        virtual void setNumberOfPoses(int numPoses) = 0;

        /** Compute unique class for each observed point
         *      (some keypoints are observed by multiple cameras but represent same 3D point)
         * @returns vector of unique classes for each point
         */
        virtual std::vector<int> assignPointClasses() = 0;

        // point format: first -- pose number, second -- local keypoint index
        virtual void insertPointsWithNewClasses(const std::vector<std::pair<int, int>> &pointsOneClass) = 0;

        virtual std::pair<int, int> getPoseNumberAndLocalIndex(int globalIndex) const = 0;

        virtual int getNumberOfPoses() const = 0;

        virtual ~IPointMatcher() = default;
    };
}
#endif
