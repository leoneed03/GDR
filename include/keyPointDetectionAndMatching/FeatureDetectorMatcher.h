//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_FEATUREDETECTORMATCHER_H
#define GDR_FEATUREDETECTORMATCHER_H

#include <vector>

#include "keyPointDetectionAndMatching/KeyPointsAndDescriptors.h"
#include "keyPointDetectionAndMatching/Match.h"
#include "keyPoints/KeyPoint2DAndDepth.h"

#include "keyPointDetectionAndMatching/ImageRetriever.h"

namespace gdr {

    /** KeyPoint SIFT detector and matcher */
    class FeatureDetectorMatcher {
    public:

        /** Process directory of images and return detected keypoints
         * @param pathsToImages contains paths as strings to image files
         * @param numOfDevicesForDetectors device indices used for multiple GPU instances, should be different
         * @returns vector where i-th element represents keypoint collection of the i-th image
         */
        virtual std::vector<std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>>>
        getKeypoints2DDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                           const std::vector<int> &numOfDevicesForDetectors) = 0;

        /** Find matches between all image keypoints (pairwise)
         * @param keyPointsDescriptorsByImageIndex contains list of image descriptors
         *      with information about detected keypoints
         * @param imageRetriever contains information about image pairs to matc
         * @param matchDevicesNumbers device indices used for multiple GPU instances, should be different
         * @returns vector where i-th element contains information about matched keypoints of the i-th image
         */
        virtual std::vector<std::vector<Match>>
        findCorrespondences(const std::vector<KeyPointsDescriptors> &keyPointsDescriptorsByImageIndex,
                            ImageRetriever &imageRetriever,
                            const std::vector<int> &matchDevicesNumbers) = 0;

        virtual ~FeatureDetectorMatcher() = default;
    };
}

#endif
