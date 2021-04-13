//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "ReaderBundleFusion.h"
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>


namespace gdr {


    ReaderBundleFusion::ReaderBundleFusion(const std::string &pathToBundleFusionDatasetToSet,
                                           const std::string &pathToSaveLikeTumToSet) :
            pathToBundleFusionDataset(pathToBundleFusionDatasetToSet),
            pathToSaveLikeTumDataset(pathToSaveLikeTumToSet) {

//        fs::path path(pathToBundleFusionDatasetToSet);
//        for (fs::directory_iterator end_dir_it, it(path); it != end_dir_it; ++it) {
//            std::cout << it->path().filename().string() << std::endl;
//        }
    }

    ReaderBundleFusion::IMAGE_TYPES ReaderBundleFusion::getImageType(const std::string &fullPath) {
        const fs::path file(fullPath);
        std::string extension = fs::extension(file);

        std::string filename = file.filename().string();
        if (extension == ".jpg") {
            return IMAGE_TYPES::RGB;
        } else if (extension == ".png") {
            return IMAGE_TYPES::DEPTH;
        }

        std::cout << filename << " of size " << filename.length() << std::endl;
        if (filename.length() >= 17) {
            if (filename.substr(13, 4) == "pose") {
                return IMAGE_TYPES::POSE;
            }
        }

        return IMAGE_TYPES::OTHER;
    }



    void ReaderBundleFusion::save(int maxNumberOfImages) {

        const fs::path directoryToSave(pathToSaveLikeTumDataset);

        fs::path rgbDirectory = directoryToSave;
        rgbDirectory.append(rgbDir);
        fs::create_directories(rgbDirectory);

        fs::path depthDirectory = directoryToSave;
        depthDirectory.append(depthDir);
        fs::create_directories(depthDirectory);


        fs::path bundleFusionRoot(pathToBundleFusionDataset);
        std::cout << "traversing " << bundleFusionRoot.string() << std::endl;
        std::vector<std::string> pathsToRGB;
        std::vector<std::string> pathsToD;
        std::vector<std::string> pathsToPoses;

        int counterRgb = 0;
        int counterD = 0;


        fs::path groundTruthFile = directoryToSave;
        groundTruthFile.append("groundtruth.txt");

        std::ofstream groundtruthTxt(groundTruthFile.string());

        for (fs::directory_iterator end_dir_it, it(bundleFusionRoot); it != end_dir_it; ++it) {
            auto imageType = getImageType(it->path().string());

            if (imageType == IMAGE_TYPES::RGB) {
                pathsToRGB.emplace_back(it->path().string());
            } else if (imageType == IMAGE_TYPES::DEPTH) {
                pathsToD.emplace_back(it->path().string());
            } else if (imageType == IMAGE_TYPES::POSE) {
                pathsToPoses.emplace_back(it->path().string());
            }
        }
        std::sort(pathsToRGB.begin(), pathsToRGB.end());
        std::sort(pathsToD.begin(), pathsToD.end());
        std::sort(pathsToPoses.begin(), pathsToPoses.end());

        assert(pathsToRGB.size() == pathsToD.size());
        assert(!pathsToRGB.empty());

        pathsToRGB.resize(maxNumberOfImages);
        pathsToD.resize(maxNumberOfImages);
        pathsToPoses.resize(maxNumberOfImages);

        for (const auto& pathToPoseInfo: pathsToPoses) {
            SE3 poseInfoSE3 = getGroundTruthPose(pathToPoseInfo);
            std::string timestamp = getTime(pathToPoseInfo);
            int timestampInt = std::stoi(timestamp);
            auto translation = poseInfoSE3.getTranslation();
            groundtruthTxt << timestampInt << ' ';

            for (int i = 0; i < 3; ++i) {
                groundtruthTxt << translation[i] << ' ';
            }
            SO3 rotation = poseInfoSE3.getSO3();
            groundtruthTxt << rotation << std::endl;

        }

        createFileTxt(pathsToRGB, "rgb");
        createFileTxt(pathsToD, "depth");

    }

    std::string ReaderBundleFusion::getTime(const std::string &imageName) {
        fs::path path(imageName);
        std::string shortName = path.filename().string();
        std::string time = shortName.substr(6, 6);

        return time;
    }

    void ReaderBundleFusion::createFileTxt(const std::vector<std::string> &pathsToImages,
                                           const std::string &imageType) {

        std::cout << "creating " << imageType << std::endl;

        fs::path tumDirTxt(pathToSaveLikeTumDataset);
        tumDirTxt.append(imageType + ".txt");
        std::ofstream output(tumDirTxt.string());


        fs::path tumDirImages(pathToSaveLikeTumDataset);
        tumDirImages.append(imageType);

        const fs::path rgbDirPathRel(imageType);

        for (const auto &pathToImageString: pathsToImages) {
            std::string timeStamp = getTime(pathToImageString);
            fs::path pathToImage = rgbDirPathRel;
            std::string nameNew = timeStamp + ".png";

            pathToImage.append(nameNew);

            {
                cv::Mat image;
                fs::path imageCurrentPath = tumDirImages;
                imageCurrentPath.append(nameNew);

                if (imageType == "depth") {

                    image = cv::imread(pathToImageString, cv::IMREAD_GRAYSCALE);
                    std::cout << "dividing depth!" << std::endl;
                    int w = 640;
                    int h = 480;
                    assert(image.cols == w);
                    assert(image.rows == h);

                    for (int i = 0; i < w; ++i) {
                        for (int j = 0; j < h; ++j) {
                            int depth = image.at<ushort>(j, i);
                            depth *= 5;

                            if (depth > 65535) {
                                depth = 0;
                            }
                            assert(depth >= 0 && depth <= 65535);
                            image.at<ushort>(j, i) = static_cast<ushort>(depth);

                        }
                    }
                } else {
                    image = cv::imread(pathToImageString);
                    std::cout << "read rgb" << std::endl;
                }
                assert(!image.empty());
                cv::imwrite(imageCurrentPath.string(), image);
            }

            output << static_cast<double>(std::stoi(timeStamp)) << ' ' << pathToImage.string() << std::endl;
        }

    }

    SE3 ReaderBundleFusion::getGroundTruthPose(const std::string &fileName) {
        std::ifstream poseFile(fileName);
        assert(poseFile.is_open());

        int size4 = 4 * 4;
        std::vector<double> m4;
        m4.reserve(size4);

        for (int i = 0; i < size4; ++i) {
            double value;
            poseFile >> value;
            std::cout << value << ' ';
            m4.push_back(value);
        }

        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> matrixSE3(m4.data());
        std::cout << "\n for " << fileName << std::endl;
        std::cout << matrixSE3 << std::endl;
        for (int i = 0; i < 3; ++i) {
            assert(matrixSE3.col(i)[3] == 0.0);
        }
        assert(matrixSE3.col(3)[3] == 1.0);

        return SE3(matrixSE3);
    }
}