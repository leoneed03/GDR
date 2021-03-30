//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <vector>

#include "keyPointDetectionAndMatching/Match.h"

namespace gdr {

    int Match::getFrameNumber() const {
        return frameNumber;
    }

    int Match::getSize() const {
        return matchNumbers.size();
    }

    std::pair<int, int> Match::getKeyPointIndexDestinationAndToBeTransformed(int matchPairIndex) const {
        return matchNumbers[matchPairIndex];
    }

    Match::Match(int newFrameNumber, const std::vector<std::pair<int, int>> &newMatchNumbers) :
            frameNumber(newFrameNumber),
            matchNumbers(newMatchNumbers) {}
}
