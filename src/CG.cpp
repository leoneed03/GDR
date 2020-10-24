#include "../include/CG.h"

#include <algorithm>

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

void MyLine(cv::Mat &img, cv::Point start, cv::Point end) {
    int thickness = 2;
    int lineType = cv::LINE_8;
    line(img,
         start,
         end,
         cv::Scalar(255, 255, 255),
         thickness,
         lineType);
}

void MyLine2(cv::Mat &img, cv::Point start, cv::Point end) {
    int thickness = 2;
    int lineType = cv::LINE_8;
    line(img,
         start,
         end,
         cv::Scalar(215, 215, 215),
         thickness,
         lineType);
}

cv::Mat CorrespondenceGraph::getEssentialMatrixTwoImagesMatched(int vertexFrom, int vertexInList) {


    const auto &frame1 = verticesOfCorrespondence[vertexFrom];
    const auto &frame2 = verticesOfCorrespondence[matches[vertexFrom][vertexInList].frameNumber];

    sift.RunSIFT(frame1.pathToRGBimage.data());
    int num1 = sift.GetFeatureNum();
    std::vector<float> descriptors1(128 * num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);
    sift.GetFeatureVector(&keys1[0], &descriptors1[0]);

    sift.RunSIFT(frame2.pathToRGBimage.data());
    int num2 = sift.GetFeatureNum();
    std::vector<float> descriptors2(128 * num2);
    std::vector<SiftGPU::SiftKeypoint> keys2(num2);
    sift.GetFeatureVector(&keys2[0], &descriptors2[0]);

    assert(num1 * 128 == descriptors1.size());
    assert(num2 * 128 == descriptors2.size());

    matcher->SetDescriptors(0, num1, &descriptors1[0]); //image 1
    matcher->SetDescriptors(1, num2, &descriptors2[0]); //image 2


    std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<SiftGPU::SiftKeypoint>> matchingKeypoints;

//    std::unique_ptr<int*> bufferPtr(new int[num1][2]);
//    std::vector<int*> matchBuff(num1, int[2] {}/*std::vector<int>(2)*/);
    int (*match_buf)[2] = new int[num1][2];
    //use the default thresholds. Check the declaration in SiftGPU.h
    int num_match = matcher->GetSiftMatch(num1, match_buf);
    matchingKeypoints.first.reserve(num_match);
    matchingKeypoints.second.reserve(num_match);

    for (int i = 0; i < num_match; ++i) {
        /* std::cout << i << " -> keypoint on the 1st image " << match_buf[i][0] << " keypoint on the 2nd image "
                   << match_buf[i][1] << std::endl;*/
        matchingKeypoints.first.emplace_back(keys1[match_buf[i][0]]);
        matchingKeypoints.second.emplace_back(keys2[match_buf[i][1]]);
    }
    assert(matchingKeypoints.first.size() == matchingKeypoints.second.size());
    delete[] match_buf;
    std::vector<cv::Point2f> leftPtsgpu, rightPtsgpu;

    cv::Mat imageWithLines = cv::imread(frame1.pathToRGBimage);
    cv::Mat imageWithLines2 = cv::imread(frame1.pathToRGBimage);
    cv::Mat imageWithLinesDepthKnown = cv::imread(frame1.pathToRGBimage);
    for (size_t i = 0; i < matchingKeypoints.first.size(); ++i) {
        cv::Point2f p1 = {matchingKeypoints.first[i].x, matchingKeypoints.first[i].y};
        cv::Point2f p2 = {matchingKeypoints.second[i].x, matchingKeypoints.second[i].y};
        leftPtsgpu.push_back(p1);
        rightPtsgpu.push_back(p2);
        MyLine(imageWithLines, p1, p2);


//        leftPtsgpu.push_back({matchingKeypoints.first[i].y, matchingKeypoints.first[i].x});
//        rightPtsgpu.push_back({matchingKeypoints.second[i].y, matchingKeypoints.second[i].x});

    }

    cv::imshow("Well..", imageWithLines);

    cv::waitKey(0);
    cv::Mat status1;
    cv::Mat Egpu = findEssentialMat(
            leftPtsgpu,     //points from left image
            rightPtsgpu,    //points from right image
            cameraMatrix,


            cv::RANSAC,  //use RANSAC for a robust solution

            0.999,        //desired solution confidence level

            1.0,          //point-to-epipolar-line threshold

            status1
    );     //binary vector for inliers






    std::vector<cv::KeyPoint> keypts1, keypts2;
    cv::Mat desc1, desc2;
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(4096);
    cv::Mat img1 = cv::imread(frame1.pathToRGBimage);
    cv::Mat img2 = cv::imread(frame2.pathToRGBimage);
    orb->detectAndCompute(img1, cv::noArray(), keypts1, desc1);
    orb->detectAndCompute(img2, cv::noArray(), keypts2, desc2);
// matching descriptors
    cv::Ptr<cv::DescriptorMatcher> matcherLocal = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> matchesLocal;
    matcherLocal->match(desc1, desc2, matchesLocal);
    std::vector<cv::Point2f> leftPts, rightPts;
    for (size_t i = 0; i < matchesLocal.size(); i++) {
        leftPts.push_back(keypts1[matchesLocal[i].queryIdx].pt);
        rightPts.push_back(keypts2[matchesLocal[i].trainIdx].pt);
        MyLine2(imageWithLines2, keypts1[matchesLocal[i].queryIdx].pt, keypts2[matchesLocal[i].trainIdx].pt);
    }


    cv::imshow("ORB ", imageWithLines2);

    cv::waitKey(0);
//    cv::destroyAllWindows();
    cv::Mat status;
    cv::Mat E = findEssentialMat(
            leftPts,     //points from left image
            rightPts,    //points from right image
            cameraMatrix,


            cv::RANSAC,  //use RANSAC for a robust solution

            0.999,        //desired solution confidence level

            1.0,          //point-to-epipolar-line threshold

            status
    );     //binary vector for inliers




    std::vector<cv::Point2f> pointsFromImage1, pointsFromImage2;
    const auto &match = matches[vertexFrom][vertexInList];
    int minSize = match.matchNumbers.size();
    pointsFromImage1.reserve(minSize);
    pointsFromImage2.reserve(minSize);

    for (int i = 0; i < minSize; ++i) {
        const auto &point1 = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first];
        const cv::Point2f p1 = {point1.x, point1.y};
//        pointsFromImage1.push_back({point1.y, point1.x});
        pointsFromImage1.push_back(p1);
        const auto &point2 = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second];
//        pointsFromImage2.push_back({point2.y, point2.x});
        const cv::Point2f p2 = {point2.x, point2.y};
        pointsFromImage2.push_back(p2);
        MyLine2(imageWithLinesDepthKnown, p1, p2);

//        auto train = point.;
    }
    for (int i = 0; i < minSize; ++i) {
    }

    cv::imshow("Known Depths ", imageWithLinesDepthKnown);

    cv::waitKey(0);
    cv::destroyAllWindows();

    assert(pointsFromImage1.size() == pointsFromImage2.size());
    if (DEBUG_PRINT)
        std::cout << "find essential matrix" << std::endl;
    auto cameraMotion = cv::findEssentialMat(pointsFromImage1,
                                             pointsFromImage2,
                                             cameraMatrix,
                                             cv::RANSAC,  //use RANSAC for a robust solution
                                             0.999,        //desired solution confidence level
                                             1.0,          //point-to-epipolar-line threshold
                                             status);
    std::cout << Egpu << std::endl;
    std::cout << "with depth is " << std::endl;
    std::cout << cameraMotion << std::endl;
    std::cout << " but prev is " << std::endl;
    std::cout << E << std::endl;
    if (DEBUG_PRINT) {
        std::cout << "found essential matrix" << std::endl << std::endl << std::endl;
    }
    return cameraMotion;
}

int CorrespondenceGraph::findEssentialMatrices() {

    for (int i = 0; i < matches.size(); ++i) {
        for (int j = 0; j < matches[i].size(); ++j) {

            const auto &match = matches[i][j];
            const auto &frameFrom = verticesOfCorrespondence[i];
            const auto &frameTo = verticesOfCorrespondence[match.frameNumber];
            if (DEBUG_PRINT) {
                std::cout << "check this " << frameFrom.index << " -> " << frameTo.index << std::endl;
            }
            assert(frameTo.index != frameFrom.index);
            auto cameraMotion = getEssentialMatrixTwoImagesMatched(i, j);
            essentialMatrices[i].push_back(essentialMatrix(cameraMotion, frameFrom, frameTo));
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

cv::Mat CorrespondenceGraph::getEssentialMatrixTwoImages(const CorrespondenceGraph &CG, const vertexCG &frame1,
                                                         const vertexCG &frame2) {
    std::vector<cv::KeyPoint> keypts1, keypts2;

    cv::Mat desc1, desc2;
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(4096);
    cv::Mat img1 = cv::imread(frame1.pathToDimage);
    cv::Mat img2 = cv::imread(frame1.pathToDimage);
    orb->detectAndCompute(img1, cv::noArray(), keypts1, desc1);
    orb->detectAndCompute(img2, cv::noArray(), keypts2, desc2);
// matching descriptors
    cv::Ptr<cv::DescriptorMatcher> matcherLocal = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> matches;
    matcherLocal->match(desc1, desc2, matches);
    std::vector<cv::Point2f> leftPts, rightPts;

    for (size_t i = 0; i < matches.size(); i++) {

        // queryIdx is the "left" image

        leftPts.push_back(keypts1[matches[i].queryIdx].pt);

        // trainIdx is the "right" image

        rightPts.push_back(keypts2[matches[i].trainIdx].pt);

    }

//robustly find the Essential Matrix

    cv::Mat status;

    cv::Mat E = findEssentialMat(

            leftPts,     //points from left image

            rightPts,    //points from right image

            CG.cameraMatrix

//
//            cv::RANSAC,  //use RANSAC for a robust solution
//
//            0.999,        //desired solution confidence level
//
//            1.0,          //point-to-epipolar-line threshold
//
//            status
    );     //binary vector for inliers





    std::vector<cv::Point2f> pointsFromImage1, pointsFromImage2;

//    std::vector<std::vector<float>> cameraMatrix_ = {{CG.fx, 0,     CG.cx},
//                                                    {0,     CG.fy, CG.cy},
//                                                    {0,     0,     1}};
//    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << CG.fx, 0, CG.cx, 0, CG.fy, CG.cy, 0, 0, 1);
    ////////////////////std::cout << CG.cameraMatrix << std::endl;
//    std::vector<float> cameraMatrix = {CG.fx, 0,     CG.cx,
//                                                    0,     CG.fy, CG.cy,
//                                                    0,     0,     1};
    int minSize = std::min(frame1.keypoints.size(), frame2.keypoints.size());
    std::cout << frame1.keypoints.size() << " and " << frame2.keypoints.size() << " min size " << minSize << std::endl;
    for (int i = 0; i < minSize; ++i) {
        auto point = frame1.keypoints[i];
        pointsFromImage1.push_back({point.y, point.x});
    }
    for (int i = 0; i < minSize; ++i) {
        auto point = frame2.keypoints[i];
        pointsFromImage2.push_back({point.y, point.x});
    }
    assert(pointsFromImage1.size() == pointsFromImage2.size());
    if (DEBUG_PRINT)
        std::cout << "find_ essential matrix" << std::endl;
    auto cameraMotion = cv::findEssentialMat(pointsFromImage1, pointsFromImage2, CG.cameraMatrix);
    std::cout << E << std::endl;
    std::cout << "our matrix " << std::endl;
    std::cout << cameraMotion << std::endl;
    if (DEBUG_PRINT)
        std::cout << "found_ essential matrix" << std::endl;
    return cameraMotion;
}

CorrespondenceGraph::CorrespondenceGraph(const std::string &pathToImageDirectoryRGB,
                                         const std::string &pathToImageDirectoryD) {

    std::vector<std::string> imagesRgb = readRgbData(pathToImageDirectoryRGB);
    std::vector<std::string> imagesD = readRgbData(pathToImageDirectoryD);

    std::cout << imagesRgb.size() << " vs " << imagesD.size() << std::endl;
    assert(imagesRgb.size() == imagesD.size());
    essentialMatrices = std::vector<std::vector<essentialMatrix>>(imagesD.size());
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
        std::vector<SiftGPU::SiftKeypoint> &keypoints = keypointAndDescriptor.first;
        std::vector<float> &descriptors = keypointAndDescriptor.second;
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
        vertexCG currentVertex(currentImage, keypointsKnownDepth, descriptorsKnownDepth, depths,
                               imagesRgb[currentImage],
                               imagesD[currentImage]);
        verticesOfCorrespondence.push_back(currentVertex);
        assert(keypointsKnownDepth.size() == depths.size());
        assert(verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].depths.size() ==
               verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].keypoints.size());
    }
    std::cout << "vertices written" << std::endl;
    matches = std::vector<std::vector<Match>>(verticesOfCorrespondence.size());


    std::cout << "trying to find corr" << std::endl;
    findCorrespondences();
    decreaseDensity();
    findEssentialMatrices();
    for (int i = 0; i < essentialMatrices.size(); ++i) {
        for (int j = 0; j < essentialMatrices[i].size(); ++j) {
            std::cout << "                          " << std::setw(4) << i << std::setw(4) << j << std::endl;
            std::cout << "                          " << std::setw(4) << essentialMatrices[i][j].vertexFrom.index
                      << std::setw(4) << essentialMatrices[i][j].vertexTo.index << std::endl;
            std::cout << essentialMatrices[i][j].innerEssentialMatrix << std::endl;
            std::cout
                    << "______________________________________________________________________________________________________"
                    << std::endl;
        }
    }
    return;


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
