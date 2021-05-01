//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "keyPoints/KeyPointMatches.h"

namespace gdr {

    KeyPointMatches::KeyPointMatches(
            const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &keyPointMatchesToSet) :
            keyPointMatches(keyPointMatchesToSet) {}

    const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &
    KeyPointMatches::getKeyPointMatchesVector() const {
        return keyPointMatches;
    }

    void KeyPointMatches::setKeyPointMatches(
            const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &keyPointMatchesToSet) {
        keyPointMatches = keyPointMatchesToSet;
    }

    int KeyPointMatches::size() const {
        return keyPointMatches.size();
    }

    std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &
    KeyPointMatches::getKeyPointMatchesVectorRef() {
        return keyPointMatches;
    }
}