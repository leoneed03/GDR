#include "../include/CG.h"

#include <algorithm>
#include <opencv2/opencv.hpp>

#define DEBUG_PRINT 1
void c(std::string output) {
    if (DEBUG_PRINT) {
        std::cout << output << std::endl;
    }
}
void c(int output) {
    if (DEBUG_PRINT) {
        std::cout << output << std::endl;
    }
}
int CorrespondenceGraph::findCorrespondences() {

    for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
        for (int j = i + 1; j < verticesOfCorrespondence.size(); ++j) {

            if (DEBUG_PRINT)
                std::cout << "currently " << i << " " << j << std::endl;
            std::vector<std::pair<int, int>> matchingNumbers = getNumbersOfMatchesKeypoints(
                    std::make_pair(verticesOfCorrespondence[i].keypoints, verticesOfCorrespondence[i].descriptors),
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

    return 0;
}

void CorrespondenceGraph::decreaseDensity() {
    for (std::vector<Match> &correspondenceList: matches) {

        std::sort(correspondenceList.begin(), correspondenceList.end(), [](const auto &lhs, const auto &rhs) {
            return lhs.matchNumbers.size() > rhs.matchNumbers.size();
        });

        if (correspondenceList.size() > maxVertexDegree) {
            std::vector<Match> newMatchList(correspondenceList.begin(), correspondenceList.begin() + maxVertexDegree);
            std::swap(correspondenceList, newMatchList);
        }
    }
}
CorrespondenceGraph::CorrespondenceGraph(const std::string &pathToImageDirectoryRGB,
                                         const std::string &pathToImageDirectoryD) {

    std::vector<std::string> imagesRgb = readRgbData(pathToImageDirectoryRGB);
    std::vector<std::string> imagesD = readRgbData(pathToImageDirectoryD);

    std::cout << imagesRgb.size() << " vs " << imagesD.size() << std::endl;
    assert(imagesRgb.size() == imagesD.size());

    std::cout << "Totally read " << imagesRgb.size() << std::endl;

    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
//    char *myargv[5] = {"-fo", "-1", "-v", "1"};
    sift.ParseParam(5, myargv);
    int support = sift.CreateContextGL();
    std::cout << "Checking" << std::endl;
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        std::cerr << "SiftGPU is not supported!" << std::endl;
    }

    boost::timer timer;

    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
    matcher->VerifyContextGL();

    c("before sift");
    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> keysDescriptorsAll =
            getKeypointsDescriptorsAllImages(sift, pathToImageDirectoryRGB);
    c("sift done");
//    int incremental = 0;
    verticesOfCorrespondence.reserve(keysDescriptorsAll.size());
    for (int currentImage = 0; currentImage < keysDescriptorsAll.size(); ++currentImage) {
        auto keypointAndDescriptor = keysDescriptorsAll[currentImage];
        c("keypoint");
        c(currentImage);
        std::vector<SiftGPU::SiftKeypoint>& keypoints = keypointAndDescriptor.first;
        std::vector<float>& descriptors = keypointAndDescriptor.second;
        std::vector<SiftGPU::SiftKeypoint> keypointsKnownDepth;
        std::vector<float> descriptorsKnownDepth;
        std::vector<int> depths;
        cv::Mat depthImage = imread(imagesD[currentImage], cv::IMREAD_GRAYSCALE);

        for (int i = 0; i < keypoints.size(); ++i) {
            int posInDescriptorVector = 128 * i;
            int currentKeypointDepth = depthImage.at<uchar>((int) keypoints[i].y, (int) keypoints[i].x);
            if (currentKeypointDepth > 0) {
                depths.push_back(currentKeypointDepth);
                keypointsKnownDepth.push_back(keypoints[i]);
                for (int descriptorCounter = 0; descriptorCounter < 128; ++descriptorCounter) {
                    descriptorsKnownDepth.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                }
            }
        }
        vertexCG currentVertex(currentImage, keypointsKnownDepth, descriptorsKnownDepth, depths, imagesRgb[currentImage],
                               imagesD[currentImage]);
        verticesOfCorrespondence.push_back(currentVertex);
        assert(keypointsKnownDepth.size() == depths.size());
        assert(verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].depths.size() == verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].keypoints.size());
    }
    std::cout << "vertices written" << std::endl;
    matches = std::vector<std::vector<Match>>(verticesOfCorrespondence.size());


    std::cout << "trying to find corr" << std::endl;
    findCorrespondences();
    decreaseDensity();

    auto testImage = verticesOfCorrespondence[10];
    cv::Mat image = cv::imread(testImage.pathToRGBimage);

    for (const auto &key: testImage.keypoints) {
//        std::cout << key.x << "::" << key.y << std::endl;
        auto &pixel = image.at<cv::Vec3b>((int) key.y, (int) key.x);
        pixel[0] = 255;
        pixel[1] = 255;
        pixel[2] = 255;

    }
    cv::imshow("Well..", image);

    cv::waitKey(0);

    cv::Mat imageD = cv::imread(testImage.pathToDimage);
    for (const auto &key: testImage.keypoints) {
//        std::cout << key.x << "::" << key.y << std::endl;
        auto &pixel = imageD.at<cv::Vec3b>((int) key.y, (int) key.x);
        pixel[0] = 255;
        pixel[1] = 255;
        pixel[2] = 255;

    }
    cv::imshow("Depth..", imageD);
    cv::waitKey(0);
    cv::destroyAllWindows();

    for (int i = 0; i < matches.size(); ++i) {
        std::cout << i << "-th frame total of  " << matches[i].size() << ": ";
        for (const auto &match: matches[i]) {
            std::cout << match.frameNumber << "_(" << match.matchNumbers.size() << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << " all matches done " << std::endl;


    c(testImage.pathToRGBimage);
    c(testImage.pathToDimage);

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
