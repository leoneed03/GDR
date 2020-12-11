#include "../include/CorrespondenceGraph.h"
#include "../include/groundTruthTransformer.h"
#include <algorithm>
#include <cmath>
#include <fstream>

#include <pcl/visualization/cloud_viewer.h>
#include <random>
#define DEBUG_PRINT 1
#define SHOW_PCL_CLOUDS 0
#define SHOW_DEPTH_IMAGES_WITH_KEYPOINTS 0

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> randMatrixUnitary(int size) {
    typedef T Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixType;

    MatrixType Q;

    int max_tries = 40;
    double is_unitary = false;

    while (!is_unitary && max_tries > 0) {
        // initialize random matrix
        Q = MatrixType::Random(size, size);

        // orthogonalize columns using the Gram-Schmidt algorithm
        for (int col = 0; col < size; ++col) {
            typename MatrixType::ColXpr colVec = Q.col(col);
            for (int prevCol = 0; prevCol < col; ++prevCol) {
                typename MatrixType::ColXpr prevColVec = Q.col(prevCol);
                colVec -= colVec.dot(prevColVec) * prevColVec;
            }
            Q.col(col) = colVec.normalized();
        }

        // this additional orthogonalization is not necessary in theory but should enhance
        // the numerical orthogonality of the matrix
        for (int row = 0; row < size; ++row) {
            typename MatrixType::RowXpr rowVec = Q.row(row);
            for (int prevRow = 0; prevRow < row; ++prevRow) {
                typename MatrixType::RowXpr prevRowVec = Q.row(prevRow);
                rowVec -= rowVec.dot(prevRowVec) * prevRowVec;
            }
            Q.row(row) = rowVec.normalized();
        }

        // final check
        is_unitary = Q.isUnitary();
        --max_tries;
    }

    if (max_tries == 0)
        eigen_assert(false && "randMatrixUnitary: Could not construct unitary matrix!");

    return Q;
}

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> randMatrixSpecialUnitary(int size) {
    typedef T Scalar;

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixType;

    // initialize unitary matrix
    MatrixType Q = randMatrixUnitary<Scalar>(size);

    // tweak the first column to make the determinant be 1
    Q.col(0) *= Eigen::numext::conj(Q.determinant());

    return Q;
}

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

int essentialMatricesAreEqual(const cv::Mat &matrixLeft, const cv::Mat &matrixRight, float epsilon) {
    if (matrixLeft.cols != matrixRight.cols) {
        return 1;
    }
    if (matrixLeft.rows != matrixRight.rows) {
        return 2;
    }
    for (int i = 0; i < matrixLeft.cols; ++i) {
        for (int j = 0; j < matrixRight.rows; ++j) {
            if (std::abs(matrixLeft.at<uchar>(j, i) - matrixRight.at<uchar>(j, i)) > epsilon) {
                return 3;
            }
        }
    }
    return 0;

}


int CorrespondenceGraph::findCorrespondences() {

    for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
        for (int j = i + 1; j < verticesOfCorrespondence.size(); ++j) {

            if (DEBUG_PRINT)
                std::cout << "currently " << i << " " << j << std::endl;
            std::vector<std::pair<int, int>> matchingNumbers = getNumbersOfMatchesKeypoints(
                    std::make_pair(verticesOfCorrespondence[i].keypoints, verticesOfCorrespondence[i].descriptors),
                    std::make_pair(verticesOfCorrespondence[j].keypoints, verticesOfCorrespondence[j].descriptors),
                    siftModule.matcher.get());
            if (DEBUG_PRINT)
                std::cout << "total matches " << matchingNumbers.size() << std::endl;
            matches[i].push_back({j, matchingNumbers});
//            for (int p = 0; p < matchingNumbers.size(); ++p) {
//                std::swap(matchingNumbers[p].first, matchingNumbers[p].second);
//            }
//            matches[j].push_back({i, matchingNumbers});
        }
    }

    return 0;
}


int CorrespondenceGraph::findCorrespondencesEveryDepth() {

    for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
        for (int j = i + 1; j < verticesOfCorrespondence.size(); ++j) {

            if (DEBUG_PRINT)
                std::cout << "currently " << i << " " << j << std::endl;
            std::vector<std::pair<int, int>> matchingNumbers = getNumbersOfMatchesKeypoints(
                    std::make_pair(verticesOfCorrespondence[i].keypoints, verticesOfCorrespondence[i].descriptors),
                    std::make_pair(verticesOfCorrespondence[j].keypoints, verticesOfCorrespondence[j].descriptors),
                    siftModule.matcher.get());
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

    siftModule.sift.RunSIFT(frame1.pathToRGBimage.data());
    int num1 = siftModule.sift.GetFeatureNum();
    std::vector<float> descriptors1(128 * num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);
    siftModule.sift.GetFeatureVector(&keys1[0], &descriptors1[0]);

    siftModule.sift.RunSIFT(frame2.pathToRGBimage.data());
    int num2 = siftModule.sift.GetFeatureNum();
    std::vector<float> descriptors2(128 * num2);
    std::vector<SiftGPU::SiftKeypoint> keys2(num2);
    siftModule.sift.GetFeatureVector(&keys2[0], &descriptors2[0]);

    assert(num1 * 128 == descriptors1.size());
    assert(num2 * 128 == descriptors2.size());

    siftModule.matcher->SetDescriptors(0, num1, &descriptors1[0]); //image 1
    siftModule.matcher->SetDescriptors(1, num2, &descriptors2[0]); //image 2


    std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<SiftGPU::SiftKeypoint>> matchingKeypoints;

//    std::unique_ptr<int*> bufferPtr(new int[num1][2]);
//    std::vector<int*> matchBuff(num1, int[2] {}/*std::vector<int>(2)*/);
    int (*match_buf)[2] = new int[num1][2];
    //use the default thresholds. Check the declaration in SiftGPU.h
    int num_match = siftModule.matcher->GetSiftMatch(num1, match_buf);
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
    cv::Mat imageWithLinesTo = cv::imread(frame2.pathToRGBimage);
    cv::Mat imageWithLines2 = cv::imread(frame1.pathToRGBimage);
    cv::Mat imageWithLinesDepthKnown = cv::imread(frame1.pathToRGBimage);
    for (size_t i = 0; i < matchingKeypoints.first.size(); ++i) {
        cv::Point2f p1 = {matchingKeypoints.first[i].x, matchingKeypoints.first[i].y};
        cv::Point2f p2 = {matchingKeypoints.second[i].x, matchingKeypoints.second[i].y};
        leftPtsgpu.push_back(p1);
        rightPtsgpu.push_back(p2);
        MyLine(imageWithLines, p1, p2);
        MyLine(imageWithLinesTo, p1, p2);

//        leftPtsgpu.push_back({matchingKeypoints.first[i].y, matchingKeypoints.first[i].x});
//        rightPtsgpu.push_back({matchingKeypoints.second[i].y, matchingKeypoints.second[i].x});

    }

    cv::imshow("Well..", imageWithLines);
    cv::waitKey(0);
    cv::imshow("Image to", imageWithLinesTo);
    cv::waitKey(0);
    cv::Mat status1;
    float lfx = 525.0;
    float lfy = 525.0;
    float lcx = 319.5;
    float lcy = 239.5;
    cv::Mat cameraMatrixLocal = (cv::Mat_<double>(3, 3) << lfx, 0, lcx, 0, lfy, lcy, 0, 0, 1);
    cv::Mat Egpu = findEssentialMat(
            leftPtsgpu,     //points from left image
            rightPtsgpu,    //points from right image
            cameraRgbd.cameraMatrix,


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
            cameraRgbd.cameraMatrix,


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
                                             cameraRgbd.cameraMatrix,
                                             cv::RANSAC,  //use RANSAC for a robust solution
                                             0.999,        //desired solution confidence level
                                             1.0,          //point-to-epipolar-line threshold
                                             status);

    std::cout << Egpu << std::endl;
    std::cout << "with depth is " << std::endl;
    std::cout << cameraMotion << std::endl;
    std::cout << " but prev is " << std::endl;
    std::cout << E << std::endl;
    assert(essentialMatricesAreEqual(Egpu, cameraMotion, 0.00001) == 0);

//    assert(essentialMatricesAreEqual(E, cameraMotion, 0.00001) == 0);
    if (DEBUG_PRINT) {
        std::cout << "found essential matrix" << std::endl << std::endl << std::endl;
    }
    return cameraMotion;
}

int CorrespondenceGraph::findTransformationRtMatrices() {

    bool f = true;
    for (int i = 0; i < matches.size(); ++i) {
        for (int j = 0; j < matches[i].size(); ++j) {

            const auto &match = matches[i][j];
            const auto &frameFrom = verticesOfCorrespondence[i];
            const auto &frameTo = verticesOfCorrespondence[match.frameNumber];
            if (DEBUG_PRINT) {
                std::cout << "check this " << frameFrom.index << " -> " << frameTo.index << std::endl;
            }
            assert(frameTo.index > frameFrom.index);
            MatrixX R, t;
            bool success = true;
            auto cameraMotion = getEssentialMatrixTwoImages(i, j, R, t, success);

            std::cout << "out of Transformation calculation" << std::endl;
            std::cout << frameFrom.index << " -> " << frameTo.index << std::endl;

            if (success) {
                tranformationRtMatrices[i].push_back(transformationRtMatrix(cameraMotion, frameFrom, frameTo, R, t));
                tranformationRtMatrices[frameTo.index].push_back(transformationRtMatrix(cameraMotion.inverse(), frameTo, frameFrom, cameraMotion.inverse().block(0, 0, 3, 3), cameraMotion.inverse().block(0, 3, 3, 1)));
            } else {
                std::cout << "/////////////////////////////////\n/////////////////////////////////\n/////////////////////////////\n NOT ENOUGH MATCHES \n/////////////////////////////////\n/////////////////////////////////\n/////////////////////////////////\n";
            }
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

int CorrespondenceGraph::findRotationsTranslations() {
    return 0;
}

int CorrespondenceGraph::findRotationTranslation(int vertexFrom, int vertexInList) {
    return 0;
}


MatrixX CorrespondenceGraph::getTransformationMatrixUmeyamaLoRANSAC(const MatrixX& firstPoints, const MatrixX& secondPoints, const int numIterations, const int numOfPoints, double inlierCoeff) {
    int dim = 3;
    int top = 10;
    if (inlierCoeff > 1) {
        inlierCoeff = 1;
    }

    if (inlierCoeff < 0) {
        inlierCoeff = 0;
    }
    assert(numOfPoints == firstPoints.cols());
    assert(firstPoints.cols() == secondPoints.cols());

    int numInliers = (int) (inlierCoeff * numOfPoints);

    std::vector<int> pointsPositions;
    pointsPositions.reserve(numOfPoints);
    for (int i = 0; i < numOfPoints; ++i) {
        pointsPositions.push_back(i);
    }
    MatrixX bestMath;





    double minError = 1e6;
    int attempt = -1;
    double mError = -1;
    std::vector<int> inlierIndices;
    srand((unsigned int) time(0));

    MatrixX cR_t_umeyama_3_points_cand;
    std::vector<int> triple;
    for (int i = 0; i < numIterations; ++i) {
        std::vector<int> p(dim, 0);
        MatrixX first3Points = MatrixX::Random(dim + 1, dim);
        MatrixX second3Points = MatrixX::Random(dim + 1, dim);
        p[0] = rand() % numOfPoints;
        p[1] = rand() % numOfPoints;
        p[2] = rand() % numOfPoints;

        while (p[0] == p[1]) {
            p[1] = rand() % numOfPoints;
        }
        while (p[0] == p[2] || p[1] == p[2]) {
            p[2] = rand() % numOfPoints;
        }
        for (int j = 0; j < p.size(); ++j) {
            first3Points.col(j) = firstPoints.col(p[j]);
            second3Points.col(j) = secondPoints.col(p[j]);
            for (int assertCounter = 0; assertCounter < dim; ++assertCounter) {
                assert(firstPoints.col(p[j])[assertCounter] == first3Points.col(j)[assertCounter]);
                assert(secondPoints.col(p[j])[assertCounter] == second3Points.col(j)[assertCounter]);
            }
        }

        MatrixX cR_t_umeyama_3_points = umeyama(first3Points.block(0, 0, dim, dim),
                                       second3Points.block(0, 0, dim, dim));
        std::sort(pointsPositions.begin(), pointsPositions.end(), [firstPoints, secondPoints, dim, cR_t_umeyama_3_points](const auto& lhs, const auto& rhs){
            auto& firstLeft = firstPoints.col(lhs);
            auto& firstRight = firstPoints.col(rhs);
            auto& secondLeft = secondPoints.col(lhs);
            auto& secondRight = secondPoints.col(rhs);
            double dist1 = 0;
            double dist2 = 0;
            auto& destLeft = cR_t_umeyama_3_points * firstLeft;
            auto& destRight = cR_t_umeyama_3_points * firstRight;
            for (int pp = 0; pp < dim; ++pp) {
                dist1 += pow(destLeft[pp] - secondLeft[pp], 2);
                dist2 += pow(destRight[pp] - secondRight[pp], 2);
            }
            return dist1 < dist2;
        });
//        for (int ii = 0; ii < top; ++ii) {
//            std::cout << std::setw(6) << pointsPositions[ii];
//        }
//        std::cout << endl;

        int quantilIndex = (int) (inlierCoeff * numOfPoints);
        MatrixX firstInlierPoints = MatrixX::Random(dim + 1, numInliers);
        MatrixX secondInlierPoints = MatrixX::Random(dim + 1, numInliers);
        for (int currentIndex = 0; currentIndex < numInliers; ++currentIndex) {
            int index = pointsPositions[currentIndex];
            firstInlierPoints.col(currentIndex) = firstPoints.col(index);
            secondInlierPoints.col(currentIndex) = secondPoints.col(index);

            assert(firstInlierPoints.col(currentIndex)[1] == firstPoints.col(index)[1]);
            assert(secondInlierPoints.col(currentIndex)[2] == secondPoints.col(index)[2]);
        }

        const auto& firstColumn = firstInlierPoints.col(std::max(numInliers - 1, 0));
        const auto& secondColumn = secondInlierPoints.col(std::max(numInliers - 1, 0));
        auto dest = cR_t_umeyama_3_points * firstColumn;
        double normError = 0;
        for (int pp = 0; pp < dim; ++pp) {
            normError += pow(dest[pp] - secondColumn[pp], 2);
        }
        MatrixX cR_t_umeyama_inlier_points = cR_t_umeyama_3_points;
        if (normError < minError) {
            cR_t_umeyama_inlier_points = umeyama(firstInlierPoints.block(0, 0, dim, numInliers),
                                                         secondInlierPoints.block(0, 0, dim, numInliers));
        }

//        double norm = 0;
//        for (int count = std::max(numInliers - 1, 0); count < numInliers; ++count) {
//            const auto& firstColumn = firstInlierPoints.col(count);
//            const auto& secondColumn = secondInlierPoints.col(count);
//            auto dest = cR_t_umeyama_inlier_points * firstColumn;
//            for (int pp = 0; pp < dim; ++pp) {
//                norm += pow(dest[pp] - secondColumn[pp], 2);
//            }
//        }

        bool info = false;

        if (info) {
            std::cout << "att " << std::setw(6) << i << " with error " << normError << std::endl;

            std::cout << "++++++++++++++++++++++++++++++++++++++\n" << " total inliers " << numInliers << std::endl;
        }
        if (normError < minError) {
            cR_t_umeyama_3_points_cand = cR_t_umeyama_3_points;
            mError = normError;
            bestMath = cR_t_umeyama_inlier_points;
            attempt = i;
            inlierIndices = pointsPositions;
            minError = normError;
            triple = p;
        }
    }
    if (DEBUG_PRINT) {
//        std::cout << random() << std::endl;
        std::cout << "cand \n" << cR_t_umeyama_3_points_cand << std::endl;
        std::cout << "RANSAC found on attempt " << attempt << " error on last \'inlier\' " << mError << std::endl;
        for (int i = 0; i < top; ++i) {
            std::cout << std::setw(6) << inlierIndices[i];
        }
        std::cout << std::endl;
        for (int i = 0; i < triple.size(); ++i) {
            std::cout << std::setw(6) << triple[i];
        }
        std::cout << std::endl;
    }
    return bestMath;
}
void CorrespondenceGraph::showKeypointsOnDephtImage(int vertexFrom) {
    auto& image = verticesOfCorrespondence[vertexFrom];
    cv::Mat depthImage = cv::imread(image.pathToDimage, cv::IMREAD_ANYDEPTH);
    std::cout << depthImage.cols << " " << depthImage.rows << std::endl;

    cv::Mat imageDepth1 ( 480, 640, CV_16UC1 );
    for (uint x = 0; x < depthImage.cols; ++x) {
//            std::cout << std::setw(7) << x << ":";
//            myfile << std::setw(7) << x << ":";
        for (uint y = 0; y < depthImage.rows; ++y) {
            auto currentDepth = depthImage.ptr<ushort>(y)[x];
            assert(currentDepth == depthImage.at<ushort>(y, x));
//                std::cout << std::setw(8) << currentDepth;
            imageDepth1.at<ushort>(y, x) = currentDepth;
//                myfile << std::setw(8) << currentDepth;
//                depthImageLow.ptr<ushort>(y)[x] = 0;

        }
//            std::cout << std::endl;
    }
    for (int i = 0; i < image.keypoints.size(); ++i) {
        int x = image.keypoints[i].x;
        int y = image.keypoints[i].y;
        std::cout << ((int) (image.depths[i] * 5000)) << " vs " << depthImage.at<ushort>(y, x) << std::endl;
        std::cout << image.depths[i] << " vs " << depthImage.at<ushort>(y, x) * 1.0 / 5000 << std::endl;
        assert(abs((image.depths[i]) - depthImage.at<ushort>(y, x) / 5000.0) < std::numeric_limits<float>::epsilon());
        imageDepth1.at<ushort>(y, x) = std::numeric_limits<ushort>::max();
    }
        cv::imshow("Made Depths ?", imageDepth1);
        cv::waitKey(0);
//        cv::imshow("Known Depths ?", depthImageS);
//        cv::waitKey(0);
//        cv::imshow("Known Depths low", depthImageLow);
//        cv::waitKey(0);
        cv::imshow("Known Depths high", depthImage);
        cv::waitKey(0);
        cv::destroyAllWindows();
}
MatrixX
CorrespondenceGraph::getEssentialMatrixTwoImages(int vertexFrom, int vertexInList, MatrixX &outR, MatrixX &outT, bool& success, double inlierCoeff) {
    MatrixX cR_t_umeyama;
    success = true;
    if (inlierCoeff >= 1.0) {
        inlierCoeff = 1.0;
    }
    if (inlierCoeff < 0) {
        success = false;
        return cR_t_umeyama;
    }
    {
        int dim = 3;
        std::vector<cv::Point2f> pointsFromImage1, pointsFromImage2;
        const auto &match = matches[vertexFrom][vertexInList];
        int minSize = match.matchNumbers.size();
        if (minSize < minNumberOfInliersAfterRobust / inlierCoeff)
        pointsFromImage1.reserve(minSize);
        pointsFromImage2.reserve(minSize);


        MatrixX firstPoints = MatrixX::Random(dim + 1, minSize);
        MatrixX secondPoints = MatrixX::Random(dim + 1, minSize);


        double mx = 1000, my = 1000, mz = 1000;
        double Mx = -1000, My = -1000, Mz = -1000;
        int num_elements = minSize;
        for (int i = 0; i < minSize; ++i) {
            {
                double x1, y1, z1;
                x1 = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first].x;
                y1 = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first].y;
                z1 = verticesOfCorrespondence[vertexFrom].depths[match.matchNumbers[i].first];

                double mirrorParameterH = verticesOfCorrespondence[vertexFrom].heightMirrorParameter;
                assert(y1 < mirrorParameterH && y1 > 0);
                y1 = mirrorParameterH - y1;

                double mirrorParameterW = verticesOfCorrespondence[vertexFrom].widthMirrorParameter;
                assert(x1 < mirrorParameterW && x1 > 0);
                x1 = mirrorParameterW - x1;


                //we have to mirror y because image coordinate system is upper-left located
                //and mirror x because standart XYZ swaps directions of OX!!!



                z1 = z1;
                x1 = 1.0 * (x1 - cameraRgbd.cx) * z1 / cameraRgbd.fx;
                y1 = 1.0 * (y1 - cameraRgbd.cy) * z1 / cameraRgbd.fy;

                if (z1 < mz) {
                    mx = x1;
                    my = y1;
                    mz = z1;
                }
                if (z1 > Mz) {
                    Mx = x1;
                    My = y1;
                    Mz = z1;
                }

                firstPoints.col(i) << x1, y1, z1, 1;

                assert(firstPoints.col(i)[0] == x1);
                assert(firstPoints.col(i)[1] == y1);
                assert(firstPoints.col(i)[2] == z1);
                assert(firstPoints.col(i)[3] == 1);
            }

            {
                double x2, y2, z2;
                x2 = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second].x;
                y2 = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second].y;
                z2 = verticesOfCorrespondence[match.frameNumber].depths[match.matchNumbers[i].second];

                double mirrorParameterH = verticesOfCorrespondence[vertexFrom].heightMirrorParameter;
                assert(y2 < mirrorParameterH && y2 >= 0);
                y2 = mirrorParameterH - y2;

                double mirrorParameterW = verticesOfCorrespondence[vertexFrom].widthMirrorParameter;
                assert(x2 < mirrorParameterW && x2 > 0);
                x2 = mirrorParameterW - x2;

                z2 = z2;
                x2 = 1.0 * (x2 - cameraRgbd.cx) * z2 / cameraRgbd.fx;
                y2 = 1.0 * (y2 - cameraRgbd.cy) * z2 / cameraRgbd.fy;

                secondPoints.col(i) << x2, y2, z2, 1;

                assert(secondPoints.col(i)[0] == x2);
                assert(secondPoints.col(i)[1] == y2);
                assert(secondPoints.col(i)[2] == z2);
                assert(secondPoints.col(i)[3] == 1);
            }
            const auto &point1 = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first];
            const cv::Point2f p1 = {point1.x, point1.y};
            pointsFromImage1.push_back(p1);
            const auto &point2 = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second];
            const cv::Point2f p2 = {point2.x, point2.y};
            pointsFromImage2.push_back(p2);
        }



        std::cout << "Points are min" << std::endl;
        std::cout << mx << " " << my << " " << mz << std::endl;

        std::cout << "Points are max" << std::endl;
        std::cout << Mx << " " << My << " " << Mz << std::endl;
        assert(mz > 0);
        assert(Mz > 0);
        std::swap(firstPoints, secondPoints); //// swap because we want to know M: first = M * second (first can be world origin -- want to make it easier to reconstruct)

        MatrixX cR_t_umeyama_1 = umeyama(firstPoints.block(0, 0, dim, num_elements),
                                         secondPoints.block(0, 0, dim, num_elements));
        MatrixX cR_t_umeyama_RANSAC = getTransformationMatrixUmeyamaLoRANSAC(firstPoints, secondPoints, numIterations, num_elements,
                                                                             inlierCoeff);
        cR_t_umeyama = cR_t_umeyama_RANSAC;
        if (DEBUG_PRINT) {
            std::cout << "simple umeyama " << std::endl;
            std::cout << cR_t_umeyama_1 << std::endl;
            std::cout << "RANSAC umeyama " << std::endl;
            std::cout << cR_t_umeyama_RANSAC << std::endl;
            std::cout << "______________________________________________________\n";
            std::cout << "______________________________________________________\n";
        }


/////////BEGIN UMEYAMA DEMONSTRATION
//    {
//        pcl::PointCloud<pcl::PointXYZRGB> cloud1;
//
//        cloud1.width = 2 * num_elements;
//        cloud1.height = 1;
//        cloud1.is_dense = false;
//        cloud1.points.resize(cloud1.width * cloud1.height);
//
//        for (size_t i = 0; i < num_elements; ++i) {
//            int r = 10;
//            int g = 10;
//            int b = 100;
//            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
//                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//            cloud1.points[i].x = secondPoints.col(i).x();
//            cloud1.points[i].y = secondPoints.col(i).y();
//            cloud1.points[i].z = secondPoints.col(i).z();
//            cloud1.points[i].rgb = rgb;
//            std::cout << "point " << i << " out of " << num_elements << std::endl;
//        }
//        for (size_t i = num_elements; i < 2 * num_elements; ++i) {
//
//            int r = 255;
//            int g = 255;
//            int b = 255;
//            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
//                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//            auto res = cR_t_umeyama * firstPoints.col(i - num_elements);
//            cloud1.points[i].x = res[0];
//            cloud1.points[i].y = res[1];
//            cloud1.points[i].z = res[2];
//            cloud1.points[i].rgb = rgb;
////        pcl::visualization::createLine();
//
//            std::cout << "point " << i << " out of " << num_elements << std::endl;
//        }
//        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&cloud1);
//        viewer.showCloud(ptrCloud);
//
//        while (!viewer.wasStopped()) {
//        }
//    }


/////////////////////END OF UMEYAMA DEMONSTRATION

        bool initClouds = false;
        if (initClouds) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFrom(parseDepthImageNoColour(
                verticesOfCorrespondence[vertexFrom].pathToDimage, cameraRgbd));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTo(parseDepthImageNoColour(
                verticesOfCorrespondence[vertexFrom].pathToDimage, cameraRgbd));
//        cloudTo->points.resize(minSize);
//        cloudTo->width = minSize;
//        cloudFrom->points.resize(minSize);
//        cloudFrom->width = minSize;
        std::cout << "cloud sizes are " << cloudTo->width << "->" << cloudFrom->width << std::endl;

        assert(cloudTo->width == cloudFrom->width);
        assert(cloudFrom->width > 0);
        for (size_t i = 0; i < cloudFrom->width; ++i) {
            int r = 10;
            int g = 10;
            int b = 100;
            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

            auto res = cR_t_umeyama * firstPoints.col(i);
            MatrixX columnOfPoints = MatrixX::Random(dim + 1, 1);
            columnOfPoints.col(0)[0] = cloudFrom->points[i].x;
            columnOfPoints.col(0)[1] = cloudFrom->points[i].y;
            columnOfPoints.col(0)[2] = cloudFrom->points[i].z;
            columnOfPoints.col(0)[3] = 1;
//            auto res = cR_t_umeyama * columnOfPoints.col(0);
            cloudFrom->points[i].x = res[0];
            cloudFrom->points[i].y = res[1];
            cloudFrom->points[i].z = res[2];
            double delta = 0;
            cloudTo->points[i].x = secondPoints.col(i)[0] + delta;
            cloudTo->points[i].y = secondPoints.col(i)[1] + delta;
            cloudTo->points[i].z = secondPoints.col(i)[2] + delta;
        }
//        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//        viewer.showCloud(cloudTo);
//
//
//        while (!viewer.wasStopped()) {
//        }
////
//
//        parseDepthImage(verticesOfCorrespondence[vertexFrom].pathToDimage, cameraRgbd);
//        exit(3);/////////////////////EXIT

//////////////////////BLOCK FOR NL ICP

//        double dist = 0.05;
//        double rans = 0.05;
//        int iter = 5;
//        bool nonLinear = true;
//            pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;
//            icp.setMaximumIterations(iter);
//            icp.setMaxCorrespondenceDistance(dist);
//            icp.setRANSACOutlierRejectionThreshold(rans);
//            icp.setInputSource(cloudFrom);
//            icp.setInputTarget(cloudTo);
//            icp.align(*cloudFrom);
//
//            if (icp.hasConverged()) {
//                std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
//                std::cout << "\nICP transformation " << icp.nr_iterations_ << " : cloud_icp -> cloud_in " << icp.convergence_criteria_->getAbsoluteMSE()<< std::endl;
//                auto transformation_matrix = icp.getFinalTransformation().cast<double>();
//                std::cout << transformation_matrix << std::endl;
//            } else {
//                PCL_ERROR ("\nICP has not converged.\n");
//                exit(-1);
//            }
//            std::cout << "before " << std::endl;
        }
        std::cout << "after " << std::endl;

        if (SHOW_PCL_CLOUDS) {
            pcl::PointCloud<pcl::PointXYZRGB> cloud1;


            cloud1.width = 2 * num_elements;
            cloud1.height = 1;
            cloud1.is_dense = false;
            cloud1.points.resize(cloud1.width * cloud1.height);

            for (size_t i = 0; i < num_elements; ++i) {
                int r = 10;
                int g = 10;
                int b = 100;
                int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                               static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                cloud1.points[i].x = secondPoints.col(i).x();
                cloud1.points[i].y = secondPoints.col(i).y();
                cloud1.points[i].z = secondPoints.col(i).z();
                cloud1.points[i].rgb = rgb;
                std::cout << "point " << i << " out of " << num_elements << std::endl;
            }
            for (size_t i = num_elements; i < 2 * num_elements; ++i) {

                int r = 255;
                int g = 255;
                int b = 255;
                int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                               static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                auto res = cR_t_umeyama * firstPoints.col(i - num_elements);
                cloud1.points[i].x = res[0];
                cloud1.points[i].y = res[1];
                cloud1.points[i].z = res[2];
                cloud1.points[i].rgb = rgb;
//        pcl::visualization::createLine();

                std::cout << "point " << i << " out of " << num_elements << std::endl;
            }
            pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&cloud1);
            if (SHOW_DEPTH_IMAGES_WITH_KEYPOINTS) {
                showKeypointsOnDephtImage(vertexFrom);
            }
            viewer.showCloud(ptrCloud);
            while (!viewer.wasStopped()) {
            }

        }

//    {
//        pcl::PointCloud<pcl::PointXYZ> cloud1;
//
//        cloud1.width = 2;
//        cloud1.height = 1;
//        cloud1.is_dense = false;
//        cloud1.points.resize(cloud1.width * cloud1.height);
//
//        {
//            int i = 400;
//            cloud1.points[0].x = secondPoints.col(i).x();
//            cloud1.points[0].y = secondPoints.col(i).y();
//            cloud1.points[0].z = secondPoints.col(i).z();
//            std::cout << "point " << i << " out of " << num_elements << std::endl;
//        }
//        {
//            int i = 400;
//            auto res = cR_t_umeyama * firstPoints.col(i);
//            cloud1.points[1].x = res[0];
//            cloud1.points[1].y = res[1];
//            cloud1.points[1].z = res[2];
////        pcl::visualization::createLine();
//sort(differences.begin(), differences.end(), [](const auto& lhs, const auto& rhs){ return lhs > rhs; });
//    double sum_diff = 0;
//    for (const auto& e: differences) {
//        std::cout << e << " ";
//        sum_diff += e;
//    }
//    sum_diff /= num_elements;
//    std::cout << std::endl << sum_diff << std::endl;
//            std::cout << "point " << i << " out of " << num_elements << std::endl;
//        }
//        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//        pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&cloud1);
//        viewer.showCloud(ptrCloud);
//
//        while (!viewer.wasStopped()) {
//        }
//    }


        std::vector<double> differences;
        for (int i = 0; i < num_elements; ++i) {
//        auto res = cR_t_umeyama * secondPoints.col(i);
//        assert(abs(pow(secondPoints.col(i).x() - res[0],2) - (secondPoints.col(i).x() - res[0]) * (secondPoints.col(i).x() - res[0])) < 0.000001);
//        double diff = sqrt(pow(firstPoints.col(i).x() - res[0],2) + pow(firstPoints.col(i).y() - res[1],2) + pow(firstPoints.col(i).z() - res[2],2));

            auto res = cR_t_umeyama * firstPoints.col(i);
            double diff = /*sqrt*/(pow(secondPoints.col(i).x() - res[0], 2) + pow(secondPoints.col(i).y() - res[1], 2) +
                               pow(secondPoints.col(i).z() - res[2], 2));
            //////Question: maybe change Eucledian distance if needed?

            differences.push_back(diff);
        }

        sort(differences.begin(), differences.end(), [](const auto &lhs, const auto &rhs) { return lhs < rhs; });
        std::cout << "__________________________________________\n";
        double sum_dif = 0;
        double sum_sq = 0;
        int numOfInliers = 0;
        int aprNumInliers = (int) (differences.size() * inlierCoeff);
        for (int i = 0; i < aprNumInliers; ++i) {
            const auto& e = differences[i];
            if (sqrt(e) < neighbourhoodRadius) {
                ++numOfInliers;
            }
            std::cout << e << " ";
            sum_dif += e;
            sum_sq += e * e;
        }
        if (numOfInliers < minNumberOfInliersAfterRobust) {
            success = false;
            return cR_t_umeyama;
        }
        sum_dif /= aprNumInliers;
        sum_sq /= aprNumInliers;
//        std::string redCode("\033[0;31m");
//        std::string resetCode("\033[0m");
        std::cout << std::endl << redCode << "MeanEuclidianError = " << sum_dif << "      D=" << sum_sq - sum_dif * sum_dif << resetCode << std::endl;
        std::cout << std::endl << redCode << "Inliers " << numOfInliers << resetCode << std::endl;


        sort(differences.begin(), differences.end());
        for (const auto &e: differences) {
            std::cout << e << " ";
        }
        std::cout << std::endl;

        std::vector<double> differences12;
        for (int i = 0; i < num_elements; ++i) {
            double diff = sqrt(pow(secondPoints.col(i).x() - firstPoints.col(i).x(), 2) +
                               pow(secondPoints.col(i).y() - firstPoints.col(i).y(), 2) +
                               pow(secondPoints.col(i).z() - firstPoints.col(i).z(), 2));
            differences12.push_back(diff);
        }
        std::cout << "__________________________________________\n";
        sort(differences12.begin(), differences12.end(), [](const auto &lhs, const auto &rhs) { return lhs > rhs; });
        for (const auto &e: differences12) {
            std::cout << e << " ";
        }
        std::cout << std::endl;
        sort(differences12.begin(), differences12.end());
        for (const auto &e: differences12) {
            std::cout << e << " ";
        }
        std::cout << std::endl;
///////////////////////////////////////////////////////////
//    exit(1);

        std::cout << "Umeyama\n" << cR_t_umeyama << std::endl;
    }
    std::cout << "Here!" << std::endl;
    return cR_t_umeyama;
}

CorrespondenceGraph::CorrespondenceGraph(const std::string &pathToImageDirectoryRGB,
                                         const std::string &pathToImageDirectoryD,
                                         float fx, float cx, float fy, float cy) : cameraRgbd({fx, cx, fy, cy}) {

//    cameraRgbd = CameraRGBD(fx, cx, fy, cy);
    std::vector<std::string> imagesRgb = readRgbData(pathToImageDirectoryRGB);
    std::vector<std::string> imagesD = readRgbData(pathToImageDirectoryD);

    std::cout << imagesRgb.size() << " vs " << imagesD.size() << std::endl;
    assert(imagesRgb.size() == imagesD.size());
    tranformationRtMatrices = std::vector<std::vector<transformationRtMatrix>>(imagesD.size());
    std::cout << "Totally read " << imagesRgb.size() << std::endl;

//    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
    char *myargv[4] = {"-fo", "-1", "-v", "1"};
    siftModule.sift.ParseParam(4, myargv);
    int support = siftModule.sift.CreateContextGL();
    std::cout << "Checking" << std::endl;
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        std::cerr << "SiftGPU is not supported!" << std::endl;
    }

    boost::timer timer;

//    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
    siftModule.matcher->VerifyContextGL();

    c("before sift");
    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> keysDescriptorsAll =
            getKeypointsDescriptorsAllImages(siftModule.sift, pathToImageDirectoryRGB);
    c("sift done");
//    int incremental = 0;
    verticesOfCorrespondence.reserve(keysDescriptorsAll.size());
    for (int currentImage = 0; currentImage < keysDescriptorsAll.size(); ++currentImage) {
        auto keypointAndDescriptor = keysDescriptorsAll[currentImage];
        c("processing");
        c(currentImage);
        std::vector<SiftGPU::SiftKeypoint> &keypoints = keypointAndDescriptor.first;
        std::vector<float> &descriptors = keypointAndDescriptor.second;
        std::vector<SiftGPU::SiftKeypoint> keypointsKnownDepth;
        std::vector<keypointWithDepth> keypointsKnownDepths;
        std::vector<float> descriptorsKnownDepth;
        std::vector<double> depths;

        cv::Mat depthImageLow = cv::imread(imagesD[currentImage], cv::IMREAD_GRAYSCALE);
        cv::Mat depthImage = cv::imread(imagesD[currentImage], cv::IMREAD_ANYDEPTH);
        cv::Mat depthImageS = cv::imread(imagesD[currentImage]);

        std::ofstream myfile;
//        myfile.open ("example.txt");
//        myfile << "Writing this to a file.\n";
//        myfile.close();
        int mDepth1 = 0, mDepthLow = 0;
        std::cout << depthImage.cols << " " << depthImage.rows << std::endl;

        cv::Mat imageDepth1 ( 480, 640, CV_16UC1 );
        for (uint x = 0; x < depthImage.cols; ++x) {
//            std::cout << std::setw(7) << x << ":";
//            myfile << std::setw(7) << x << ":";
            for (uint y = 0; y < depthImage.rows; ++y) {
                auto currentDepth = depthImage.ptr<ushort>(y)[x];
                assert(currentDepth == depthImage.at<ushort>(y, x));
//                imageDepth1
                if (mDepth1 < currentDepth) {
                    mDepth1 = currentDepth;
                }
                if (mDepthLow < depthImageLow.ptr<ushort>(y)[x]) {
                    mDepthLow = currentDepth;
                }
//                std::cout << std::setw(8) << currentDepth;
                imageDepth1.at<ushort>(y, x) = 65535 - currentDepth;
//                myfile << std::setw(8) << currentDepth;
//                depthImageLow.ptr<ushort>(y)[x] = 0;

            }
//            std::cout << std::endl;
        }
//        myfile.close();
//        exit(0);
//        depthImage.at<uint>(10,50) = 255;
        int x = 200, y = 200;
        std::cout << "depth1 " << depthImage.depth() << " and " << depthImage.channels() << std::endl;
        std::cout << "depthLow " << depthImageLow.depth() << std::endl;
        std::cout << "full value is  ?" << depthImageS.ptr<ushort>(y)[x] << std::endl;
        std::cout << "full value is " << depthImage.ptr<ushort>(y)[x] << std::endl;
        std::cout << "low value is " << depthImageLow.ptr<ushort>(y)[x] << std::endl;
        std::cout << "Max depth  " << mDepth1 << " vs low " << mDepthLow << std::endl;
//        cv::imshow("Made Depths ?", imageDepth1);
//        cv::waitKey(0);
//        cv::imshow("Known Depths ?", depthImageS);
//        cv::waitKey(0);
//        cv::imshow("Known Depths low", depthImageLow);
//        cv::waitKey(0);
//        cv::imshow("Known Depths high", depthImage);
//        cv::waitKey(0);
//        cv::destroyAllWindows();
        for (int i = 0; i < keypoints.size(); ++i) {
            int posInDescriptorVector = 128 * i;
            int currentKeypointDepth = depthImage.at<ushort>(keypoints[i].y, keypoints[i].x);

            if (currentKeypointDepth > 0) {
                assert(currentKeypointDepth < 66000);
                depths.push_back(currentKeypointDepth / 5000.0);
                keypointsKnownDepth.push_back(keypoints[i]);
                std::vector<float> currentDescriptors;
                for (int descriptorCounter = 0; descriptorCounter < 128; ++descriptorCounter) {
                    descriptorsKnownDepth.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                    currentDescriptors.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                }
                keypointsKnownDepths.push_back(
                        {keypoints[i], currentKeypointDepth / 5000.0, currentDescriptors});
            }
        }
        VertexCG currentVertex(currentImage, keypointsKnownDepths, keypointsKnownDepth, descriptorsKnownDepth, depths,
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
    findTransformationRtMatrices();

    for (int i = 0; i < tranformationRtMatrices.size(); ++i) {
        for (int j = 0; j < tranformationRtMatrices[i].size(); ++j) {
//            std::cout << "                          " << std::setw(4) << i << std::setw(4) << j << std::endl;
            std::cout << "                          " << std::setw(4) << tranformationRtMatrices[i][j].vertexFrom.index
                      << std::setw(4) << tranformationRtMatrices[i][j].vertexTo.index << std::endl;
            std::cout << tranformationRtMatrices[i][j].innerTranformationRtMatrix << std::endl;
            std::cout << "Rotation " << std::endl;
            std::cout << tranformationRtMatrices[i][j].R << std::endl;
            std::cout << "translation " << std::endl;
            std::cout << tranformationRtMatrices[i][j].t << std::endl;
            std::cout
                    << "______________________________________________________________________________________________________"
                    << std::endl;
        }
    }


    std::string poseFile = relativePose;
    {
        std::ofstream file(poseFile);
        int numPoses = tranformationRtMatrices.size();
        for (int i = 0; i < numPoses; ++i) {
            std::string s1 = "VERTEX_SE3:QUAT ";
            std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
            file << s1 + s2;
        }
        std::set<std::string> strings;
        for (int i = 0; i < tranformationRtMatrices.size(); ++i) {
            for (int j = 0; j < tranformationRtMatrices[i].size(); ++j) {
                if (i >= tranformationRtMatrices[i][j].vertexTo.index) {
                    continue;
                }
                std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";
                std::string edgeId = "EDGE_SE3:QUAT " + std::to_string(tranformationRtMatrices[i][j].vertexTo.index) + " " + std::to_string(i) + " ";
                auto translationVector = tranformationRtMatrices[i][j].t;
//                std::string edgeWithTranslation = edgeId + std::to_string(translationVector.col(0)[0]) + " "  + std::to_string(translationVector.col(0)[1]) + " " + std::to_string(translationVector.col(0)[2]) + " ";
                std::string edgeWithTranslation = edgeId + "0.0 0.0 0.0 ";
                const auto& R = tranformationRtMatrices[i][j].R;
//                MatrixX R = randMatrixSpecialUnitary<Scalar>(3);

                Eigen::Matrix3f Rf;
                Rf << R.row(0)[0], R.row(0)[1], R.row(0)[2], R.row(1)[0], R.row(1)[1], R.row(1)[2], R.row(2)[0], R.row(
                        2)[1], R.row(2)[2];

                Eigen::Quaternionf qR(Rf);
                int space = 12;
                std::vector<double> vectorDataRotations = {qR.x(), qR.y(), qR.z(), qR.w()};
                std::string edgeTotal = edgeWithTranslation + std::to_string(qR.x()) + " " + std::to_string(qR.y()) + " " + std::to_string(qR.z()) + " "
                + std::to_string(qR.w()) + noise + "\n";
                if (strings.find(edgeTotal) != strings.end()) {
                    std::cerr << "Duplicate " << i << " " << j << " j as " << tranformationRtMatrices[i][j].vertexFrom.index << std::endl;
                    std::cout << "ERROR";
                    exit(2);
                }
                strings.insert(edgeTotal);
                file << edgeTotal;
            }
        }
    }
    printConnections(std::cout);

    std::cout << "first print successfull" << std::endl;
    rotationAverager::shanonAveraging(poseFile, absolutePose);

    std::cout << "Shonan averaging successfull" << std::endl;
    std::vector<std::vector<double>> quaternions = parseAbsoluteRotationsFile(absolutePose);

    std::cout << "read quaternions successfull" << std::endl;
    std::vector<MatrixX> absoluteRotations = getRotationsFromQuaternionVector(quaternions);

    std::cout << "get Rotations from quaternions successfull" << std::endl;
    for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
        verticesOfCorrespondence[i].setRotation(absoluteRotations[i]);
    }

    std::cout << "set Rotations in vertices successfull" << std::endl;
    bfs(0);

    std::cout << "bfs successfull" << std::endl;
    printConnections(std::cout);

    return;

//
//    auto testImage = verticesOfCorrespondence[10];
//    cv::Mat image = cv::imread(testImage.pathToRGBimage);
//
//    for (const auto &key: testImage.keypoints) {
////        std::cout << key.x << "::" << key.y << std::endl;
//        auto &pixel = image.at<cv::Vec3b>((int) key.y, (int) key.x);
//        pixel[0] = 255;
//        pixel[1] = 255;
//        pixel[2] = 255;
//
//    }
//    cv::imshow("Well..", image);
//
//    cv::waitKey(0);
//
//    cv::Mat imageD = cv::imread(testImage.pathToDimage);
//    for (const auto &key: testImage.keypoints) {
////        std::cout << key.x << "::" << key.y << std::endl;
//        auto &pixel = imageD.at<cv::Vec3b>((int) key.y, (int) key.x);
//        pixel[0] = 255;
//        pixel[1] = 255;
//        pixel[2] = 255;
//
//    }
//    cv::imshow("Depth..", imageD);
//    cv::waitKey(0);
//    cv::destroyAllWindows();
//
//    for (int i = 0; i < matches.size(); ++i) {
//        std::cout << i << "-th frame total of  " << matches[i].size() << ": ";
//        for (const auto &match: matches[i]) {
//            std::cout << match.frameNumber << "_(" << match.matchNumbers.size() << ") ";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << " all matches done " << std::endl;
//
//
//    c(testImage.pathToRGBimage);
//    c(testImage.pathToDimage);
//
//    int a = 2, b = 17;
//
//    {
//        ++a;
//        ++b;
//
//        auto matchingKeypoints = getMatchesKeypoints(
//                std::make_pair(verticesOfCorrespondence[a].keypoints, verticesOfCorrespondence[a].descriptors),
//                std::make_pair(verticesOfCorrespondence[b].keypoints, verticesOfCorrespondence[b].descriptors),
//                siftModule.matcher.get());
//        std::cout << "totally matched matchingKeypoints: " << matchingKeypoints.first.size() << " and "
//                  << matchingKeypoints.second.size() << std::endl;
//        --a;
//        --b;
//    }
//    {
//        std::cout << a << " match " << b << std::endl;
//        auto matchingKeypoints = getMatchesKeypoints(keysDescriptorsAll[a], keysDescriptorsAll[b],
//                                                     siftModule.matcher.get());
//        std::cout << "totally matched matchingKeypoints: " << matchingKeypoints.first.size() << " and "
//                  << matchingKeypoints.second.size() << std::endl;
//    }
//
//    {
//        a = 17;
//        b = 33;
//
//        std::cout << a << " match " << b << std::endl;
//        auto matchingKetpoints = getMatchesKeypoints(keysDescriptorsAll[a], keysDescriptorsAll[b],
//                                                     siftModule.matcher.get());
//        std::cout << "totally matched matchingKeypoints: " << matchingKetpoints.first.size() << " and "
//                  << matchingKetpoints.second.size() << std::endl;
//    }
//    std::cout << a << " matched " << b << std::endl;
////    delete matcher;

};


void CorrespondenceGraph::printConnections(std::ostream& os, int space) {

//    os << "======================POSES BEFORE=======================\n" << std::endl;
//    for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
//        std::cout << "Pose number: " << i << std::endl;
//        std::cout << verticesOfCorrespondence[i].absoluteRotationTranslation;
//        std::cout << "\n_________________________________________________________________\n";
//    }
    int counter = 0;
    int counterSquared = 0;
    os << "EDGES of the Correspondence Graph:" << std::endl;
    for (int i = 0; i < tranformationRtMatrices.size(); ++i) {
        os << std::setw(space / 5) << i << ":";
        counter += tranformationRtMatrices[i].size();
        counterSquared += tranformationRtMatrices[i].size() * tranformationRtMatrices[i].size();
        for (int j = 0; j < tranformationRtMatrices[i].size(); ++j) {
            const transformationRtMatrix& e = tranformationRtMatrices[i][j];
            assert(i == e.vertexFrom.index);
            os << std::setw(space / 2) << e.vertexTo.index << ",";
        }
        os << std::endl;
    }
    os << "average number of edges " << counter / tranformationRtMatrices.size() << std::endl;

    os << "sq D " << sqrt(counterSquared * 1.0 / tranformationRtMatrices.size() - pow(counter * 1.0 / tranformationRtMatrices.size(), 2)) << std::endl;
    os << "======================NOW 4*4 Matrices=======================\n" << std::endl;

    os << "======================++++++++++++++++=======================\n" << std::endl;
    for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
        std::cout << "Pose number: " << i << std::endl;
        std::cout << verticesOfCorrespondence[i].absoluteRotationTranslation;
        std::cout << "\n_________________________________________________________________\n";
    }
}

std::vector<int> CorrespondenceGraph::bfs(int currentVertex) {
    std::vector<bool> visited(verticesOfCorrespondence.size(), false);
    std::vector<int> preds(verticesOfCorrespondence.size(), -1);
    std::queue<int> queueVertices;
    queueVertices.push(currentVertex);
    assert(verticesOfCorrespondence.size() == tranformationRtMatrices.size());
    while (!queueVertices.empty()) {
        int vertex = queueVertices.front();
        std::cout << " entered vertex " << vertex << std::endl;
        queueVertices.pop();
        assert(vertex < visited.size() && vertex >= 0);
        visited[vertex] = true;

        for (int i = 0; i < tranformationRtMatrices[vertex].size(); ++i) {
            int to = tranformationRtMatrices[vertex][i].vertexTo.index;
            if (!visited[to]) {
                queueVertices.push(to);
//                visited[to] = true;
                assert(preds[to] == -1);
                preds[to] = vertex;


                ///// get absolute Rotation (R) and Translation (t) with given predecessor Vertex and Relative R & t
                const MatrixX& predAbsoluteRt = verticesOfCorrespondence[vertex].absoluteRotationTranslation;
                MatrixX predR = predAbsoluteRt.block(0, 0, 3, 3);
                MatrixX predT = predAbsoluteRt.block(0, 3, 3, 1);

                const MatrixX& relativeRt = tranformationRtMatrices[vertex][i].innerTranformationRtMatrix;
                MatrixX relR = relativeRt.block(0, 0, 3, 3);
                MatrixX relT = relativeRt.block(0, 3, 3, 1);

                MatrixX& newAbsoluteRt = verticesOfCorrespondence[to].absoluteRotationTranslation;
                MatrixX newAbsoluteR = newAbsoluteRt.block(0, 0, 3, 3);
                MatrixX newAbsoluteT = predT;

                newAbsoluteT = predR * relT + predT;
                for (int counter = 0; counter < 3; ++counter) {
                    newAbsoluteRt.col(3)[counter] = newAbsoluteT.col(0)[counter];

                }
//                relativeRt.block(0, 3, 3, 1) = newAbsoluteT;


            }
        }
    }
    return preds;



}