//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ISIFTMODULE_H
#define GDR_ISIFTMODULE_H

#include <vector>
#include <tbb/concurrent_vector.h>

#include "keyPointDetectionAndMatching/KeyPointsAndDescriptors.h"
#include "keyPointDetectionAndMatching/Match.h"
#include "keyPoints/KeyPoint2D.h"

namespace gdr {

    /** KeyPoint SIFT detector and matcher */
    class ISiftModule {
    public:

        /** Process directory of images and return detected keypoints
         * @param pathsToImages contains paths as strings to image files
         * @param numOfDevicesForDetectors device indices used for multiple GPU instances, should be different
         * @returns vector where i-th element represents keypoint collection of the i-th image
         */
        virtual std::vector<std::pair<std::vector<KeyPoint2D>, std::vector<float>>>
        getKeypoints2DDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                           const std::vector<int> &numOfDevicesForDetectors = {0}) = 0;

        /** Find matches between all image keypoints (pairwise)
         * @param keyPointsDescriptorsByImageIndex contains list of image descriptors
         *      with information about detected keypoints
         * @param matchDevicesNumbers device indices used for multiple GPU instances, should be different
         * @returns vector where i-th element contains information about matched keypoints of the i-th image
         */
        virtual std::vector<std::vector<Match>>
        findCorrespondences(const std::vector<KeyPointsDescriptors> &keyPointsDescriptorsByImageIndex,
                            const std::vector<int> &matchDevicesNumbers = {0}) = 0;

        virtual ~ISiftModule() = default;
    };
}

#endif
