//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#ifndef GDR_KEYPOINTMATCHES_H
#define GDR_KEYPOINTMATCHES_H

#include "keyPoints/KeyPointInfo.h"

#include <vector>

namespace gdr {

    /** Store information about matched keypoints between two images */
    class KeyPointMatches {

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> keyPointMatches;

    public:
        /**
         *
         * @param keyPointMatches contains N vectors for N matched keypoint pairs
         *      each pair is stored as vector where each element represents one keypoint as pair
         *          first pair element: {imageIndex, keypoint local index in image decriptor},
         *          second pair element: {KeyPointInfo: information about keypoint's image coordinates,
         *                                          depth in meters and scale, orietation}
         *      first stored point is from transformed image and second -- from destination image
         */
        KeyPointMatches(const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &keyPointMatches);

        KeyPointMatches() = default;

        void setKeyPointMatches(
                const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &keyPointMatchesToSet);

        const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &getKeyPointMatchesVector() const;

        int size() const;

    };
}


#endif
