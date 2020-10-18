#include "../include/CG.h"

#include <algorithm>

#define DEBUG_PRINT 1

int CorrespondenceGraph::findCorrespondences() {
    for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
        for (int j = i + 1; j < verticesOfCorrespondence.size(); ++j) {

            if (DEBUG_PRINT)
            std::cout << "currently " << i << " " << j << std::endl;
            std::vector<std::pair<int,int>> matchingNumbers = getNumbersOfMatchesKeypoints(std::make_pair(verticesOfCorrespondence[i].keypoints, verticesOfCorrespondence[i].descriptors),
                                                                                           std::make_pair(verticesOfCorrespondence[j].keypoints, verticesOfCorrespondence[j].descriptors),
                                                                                           matcher.get());
            if (DEBUG_PRINT)
            std::cout << "total matches " << matchingNumbers.size() << std::endl;
            matches[i].push_back({j, matchingNumbers});
            for (int p = 0; p < matchingNumbers.size(); ++p) {
                std::swap(matchingNumbers[p].first, matchingNumbers[p].second);
            }
            matches[j].push_back({i, matchingNumbers});
        }
    }

    for (std::vector<Match>& correspondenceList: matches) {
        std::sort(correspondenceList.begin(), correspondenceList.end(), [](const auto& lhs, const auto& rhs){
            return lhs.matchNumbers.size() > rhs.matchNumbers.size();
        });
        if (correspondenceList.size() > maxVertexDegree) {
//            correspondenceList.resize(maxVertexDegree);
        }
    }

    return 0;
}
CorrespondenceGraph::CorrespondenceGraph(const std::string &pathToImageDirectoryRGB,
                                         const std::string &pathToImageDirectoryD) {

    std::vector<std::string> imagesRgb = readRgbData(pathToImageDirectoryRGB);
    std::cout << "Totally read " << imagesRgb.size() << std::endl;

    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
//    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
    sift.ParseParam(5, myargv);
    int support = sift.CreateContextGL();
    std::cout << "Checking" << std::endl;
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        std::cerr << "SiftGPU is not supported!" << std::endl;
    }

    boost::timer timer;

    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(4096));
    matcher->VerifyContextGL();
    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> keysDescriptorsAll =
            getKeypointsDescriptorsAllImages(sift, pathToImageDirectoryRGB);
    int incremental = 0;
    for (auto keypointAndDescriptor: keysDescriptorsAll) {
        std::vector<SiftGPU::SiftKeypoint> k1 = keypointAndDescriptor.first;
        vertexCG currentVertex(incremental, k1, keypointAndDescriptor.second, imagesRgb[incremental], imagesRgb[incremental]);
        verticesOfCorrespondence.push_back(
                vertexCG(incremental, keypointAndDescriptor.first, keypointAndDescriptor.second, imagesRgb[incremental], imagesRgb[incremental]));
        ++incremental;
    }
    std::cout << "vertices written" << std::endl;
    matches = std::vector<std::vector<Match>>(verticesOfCorrespondence.size());


    std::cout << "trying to find corr" << std::endl;
    findCorrespondences();
    for (int i = 0; i < matches.size(); ++i) {
        std::cout << i << "-th frame total of  " << matches[i].size() << ": ";
        for (const auto& match: matches[i]) {
            std::cout << match.frameNumber << "_(" << match.matchNumbers.size() << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << " all matches done " << std::endl;
    int a = 2, b = 17;

    {
        ++a;
        ++b;

        auto matchingKeypoints = getMatchesKeypoints(
                std::make_pair(verticesOfCorrespondence[a].keypoints, verticesOfCorrespondence[a].descriptors),
                std::make_pair(verticesOfCorrespondence[b].keypoints, verticesOfCorrespondence[b].descriptors),
                matcher.get());
        std::cout << "totally matched matchingKeypoints: " << matchingKeypoints.first.size() << " and "
                  << matchingKeypoints.second.size() << std::endl;
        --a;
        --b;
    }
    {
        std::cout << a << " match " << b << std::endl;
        auto matchingKeypoints = getMatchesKeypoints(keysDescriptorsAll[a], keysDescriptorsAll[b], matcher.get());
        std::cout << "totally matched matchingKeypoints: " << matchingKeypoints.first.size() << " and "
                  << matchingKeypoints.second.size() << std::endl;
    }

    {
        a = 17;
        b = 33;

        std::cout << a << " match " << b << std::endl;
        auto matchingKetpoints = getMatchesKeypoints(keysDescriptorsAll[a], keysDescriptorsAll[b], matcher.get());
        std::cout << "totally matched matchingKeypoints: " << matchingKetpoints.first.size() << " and "
                  << matchingKetpoints.second.size() << std::endl;
    }
    std::cout << a << " matched " << b << std::endl;
//    delete matcher;

};
