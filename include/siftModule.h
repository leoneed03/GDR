//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SIFT_H
#define GDR_SIFT_H

#include <vector>
#include <memory>
#include <tbb/concurrent_queue.h>

#include "VertexCG.h"
#include "SiftGPU.h"

namespace gdr {

    struct Match {
        int frameNumber;
        std::vector<std::pair<int, int>> matchNumbers;

        Match(int newFrameNumber, const std::vector<std::pair<int, int>> &newMatchNumbers) :
                frameNumber(newFrameNumber),
                matchNumbers(newMatchNumbers) {};
    };

    struct SiftModule {

        std::unique_ptr<SiftMatchGPU> matcher;
        int maxSift = 4096;

        SiftModule();

        void siftParseParams(SiftGPU* sift, std::vector<char *> &siftGpuArgs);

        static void
        getKeypointsDescriptorsOneImage(SiftGPU* detectorSift,
                                        tbb::concurrent_queue<std::pair<std::string, int>>& pathsToImagesAndImageIndices,
                                        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> &keyPointsAndDescriptorsByIndex,
                                        std::mutex& output);

        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
        getKeypointsDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                         const std::vector<int>& numOfDevicesForDetectors = {0});

        std::vector<std::vector<Match>> findCorrespondences(const std::vector<VertexCG>& verticesToBeMatched);
    };
}

#endif
