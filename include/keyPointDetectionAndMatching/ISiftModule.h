//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ISIFTMODULE_H
#define GDR_ISIFTMODULE_H

#include <tbb/concurrent_vector.h>
#include "keyPoints/KeyPoint2D.h"

namespace gdr {

    struct Match {
        int frameNumber;
        std::vector<std::pair<int, int>> matchNumbers;

        Match(int newFrameNumber, const std::vector<std::pair<int, int>> &newMatchNumbers) :
                frameNumber(newFrameNumber),
                matchNumbers(newMatchNumbers) {};
    };


    class ISiftModule {
    public:

        virtual std::vector<std::pair<std::vector<KeyPoint2D>, std::vector<float>>>
        getKeypoints2DDescriptorsAllImages(const std::vector <std::string> &pathsToImages,
                                         const std::vector<int> &numOfDevicesForDetectors = {0}) = 0;

        virtual std::vector<std::vector<Match>>
        findCorrespondences(const std::vector<VertexCG> &verticesToBeMatched,
                            const std::vector<int> &matchDevicesNumbers = {0}) = 0;
        virtual ~ISiftModule() = default;
    };
}
#endif
