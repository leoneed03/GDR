#include "../include/groundTruthTransformer.h"
#include <fstream>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define spaceIO (15)

//#include <filesystem.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/LU> // required for MatrixBase::determinant
#include <Eigen/SVD> // required for SVD
#include <iomanip>
#include <set>

void putAligned(std::ofstream &of, const std::vector<double> &val) {
    for (const auto &vals: val) {
        (of << std::setw(12) << vals << ' ');
    }
}

int GTT::makeRotationsRelative(const std::string &pathToGroundTruth, const std::string &pathToRelativeGroundTruth) {
    std::ifstream in(pathToGroundTruth);
    std::ofstream out(pathToRelativeGroundTruth);
    int numOfEmptyLines = 3;
    int numbersInLine = 8;
    std::vector<double> stamp0;
    std::vector<double> prevCoordinates = {0, 0, 0};
    bool isZero = true;

    if (in) {
        for (int i = 0; i < 3; ++i) {
            std::string s;
            std::getline(in, s);
        }
        double currVal = -1;
        while (true) {

            std::vector<double> stamps;
            for (int i = 0; i < numbersInLine; ++i) {
                if (in >> currVal) {
                    stamps.push_back(currVal);
                } else {
                    return 1;
                }
            }
            assert(stamps.size() == numbersInLine);
            if (isZero) {
                stamp0 = stamps;
                isZero = false;
            }

            std::vector<double> vectorData0 = {stamp0[4], stamp0[5], stamp0[6], stamp0[7]};
            std::vector<double> vectorData = {stamps[4], stamps[5], stamps[6], stamps[7]};
            Eigen::Quaterniond qd(vectorData.data());
            Eigen::Quaterniond q0(vectorData0.data());
            auto qRelative = q0.inverse() * qd;
            std::vector<double> toStream = {stamps[0] - stamp0[0], stamps[1] - prevCoordinates[0],
                                            stamps[2] - prevCoordinates[1], stamps[3] - prevCoordinates[2],
                                            qRelative.x(), qRelative.y(), qRelative.z(), qRelative.w()};
            putAligned(out, toStream);
            out << std::endl;
            prevCoordinates = {stamps[1], stamps[2], stamps[3]};
            //out << stamps[0] - stamp0[0] << ' ' << stamps[1] << ' ' << stamps[2] << ' ' << stamps[3] << ' ' << qRelative.x() << ' ' << qRelative.y() << ' ' << qRelative.z() << ' ' << qRelative.w() << std::endl;

        }

    }
}

std::vector<std::string> readData(std::string pathToRGB) {
    DIR *pDIR;
    struct dirent *entry;
    std::vector<std::string> RgbImages;
    std::cout << "start reading" << std::endl;
    if ((pDIR = opendir(pathToRGB.data())) != nullptr) {
        int imageCounter = 0;
        while ((entry = readdir(pDIR)) != nullptr) {
            if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") {
                continue;
            }
            std::string rPathToRgbImage = entry->d_name;
//            std::cout << ++imageCounter << ": " << absolutePathToRgbImage << "\n";
            RgbImages.emplace_back(rPathToRgbImage);
        }
        closedir(pDIR);
    } else {
        std::cout << "Unable to open" << std::endl;
    }
    std::sort(RgbImages.begin(), RgbImages.end());
    for (int i = 0; i < RgbImages.size(); ++i) {
        std::cout << i + 1 << "::" << RgbImages[i] << std::endl;
    }
    return RgbImages;
}

std::pair<std::vector<std::string>, std::vector<std::string>>
GTT::makeRotationsRelativeAndExtractImages(const std::string &pathToGroundTruth, const std::string &pathToRGB,
                                           const std::string &pathToD, const std::string &pathOutDirectory,
                                           const std::string &timeInfo,
                                           const std::set<int> indices) {
    std::ifstream in(pathToGroundTruth);
    std::string outRGB = pathOutDirectory + "/rgb";
    std::string outD = pathOutDirectory + "/depth";

    boost::filesystem::create_directory(pathOutDirectory);
    namespace fs = boost::filesystem;
    fs::path path_to_remove(pathOutDirectory);
    for (fs::directory_iterator end_dir_it, it(path_to_remove); it != end_dir_it; ++it) {
        fs::remove_all(it->path());
    }
    boost::filesystem::create_directory(outD);
    boost::filesystem::create_directory(outRGB);
//    std::string pathToRelativeGroundTruth = pathOutDirectory + "/groundtruth.txt";
//    std::ofstream out(pathToRelativeGroundTruth);
    int numOfEmptyLines = 3;
    int numbersInLine = 8;
    std::vector<double> stamp0;
    std::vector<double> prevCoordinates = {0, 0, 0};
    bool isZero = true;
    int counter = -1;

//    if (in) {
//        for (int i = 0; i < 3; ++i) {
//            std::string s;
//            std::getline(in, s);
//            out << s << std::endl;
//        }
//        double currVal = -1;
//        for (std::string s; std::getline(in, s);) {
//            ++counter;
//            if (indices.find(counter) != indices.end()) {
//                out << s << std::endl;
//            }
//        }
//        makeRotationsRelative(pathToRelativeGroundTruth, pathOutDirectory + "/groundtruthR.txt");
//    }
    std::vector<std::string> rgbDataR = readData(pathToRGB);
    std::vector<std::string> dDataR = readData(pathToD);
    std::vector<std::string> rgbData = readRgbData(pathToRGB);
    std::vector<std::string> dData = readRgbData(pathToD);
    assert(rgbData.size() == dData.size());
//    std::string directory = pathOutDirectory + "/RGB_D";
//    boost::filesystem::create_directory(directory);

    std::vector<std::string> onlyTakenRGB;
    int cntr = 0;
    for (const auto &e: indices) {
        if (e >= rgbData.size()) {
            break;
        }
        std::string toRGB = outRGB + "/" + std::to_string(cntr) + "_" + rgbDataR[e];
        std::string toD = outD + "/" + std::to_string(cntr) + "_" + dDataR[e];
        boost::filesystem::copy_file(rgbData[e], toRGB);
        boost::filesystem::copy_file(dData[e], toD);
        onlyTakenRGB.push_back(rgbDataR[e]);
        ++cntr;
    }
    writeInfo(onlyTakenRGB, timeInfo, pathToGroundTruth, pathOutDirectory + "/groundtruth_new.txt", indices);
    return {rgbDataR, dDataR};
//    makeRotationsRelative();
}

std::vector<double>
GTT::createTimestamps(const std::vector<std::string> &rgb,
                      const std::string &pathTimeRGB, const std::string &pathToGroundTruth,
                      const std::set<int> &indices) {
    std::map<std::string, double> rgbToTime;
    int skipNum = 3;
    std::vector<double> timeStamps;
    std::ifstream in(pathTimeRGB);
    std::vector<double> stamp0;
    std::vector<double> prevCoordinates = {0, 0, 0};
    bool isZero = true;
    int index = 0;

    if (in) {
        for (int i = 0; i < skipNum; ++i) {
            std::string s;
            std::getline(in, s);
        }
        double currVal = -1;
        std::string currPath;

        while (true) {
            std::vector<double> stamps;
            if (timeStamps.size() == rgb.size()) {
                return timeStamps;
            }
            if (in >> currVal && in >> currPath) {
                if (currPath == ("rgb/" + rgb[index])) {
                    rgbToTime[rgb[index]] = currVal;
                    timeStamps.push_back(currVal);
                    ++index;
                } else {
//                    std::cout << currPath << " _____ " << ("rgb/" + rgb[index]) << std::endl;
                }
            } else {

                if (timeStamps.size() == rgb.size()) {
                    return timeStamps;
                } else {
                    std::cout << timeStamps.size() << " vs " << rgb.size() << std::endl;
                    std::cout << timeStamps[timeStamps.size() - 1] << " vs " << rgb[rgb.size() - 1] << std::endl;
                    assert(false);
                }
            }
            if (index >= rgb.size()) {
//                std::cout << timeStamps.size() << " " << index << " " << rgb.size() << std::endl;
            }
            assert(index <= rgb.size());
        }
    }
}

std::vector<std::vector<double>>
GTT::getGroundTruth(const std::string &pathToGroundTruth, const std::vector<double> &timeStamps) {
    std::ifstream in(pathToGroundTruth);
    int numOfEmptyLines = 3;
    int numbersInLine = 8;
    double time = -1;
    std::vector<double> stamp0;
    std::vector<double> prevCoordinates = {0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<std::vector<double>> coordAndQuat;

    bool isZero = true;

    int index;

    if (in) {
        for (int i = 0; i < numOfEmptyLines; ++i) {
            std::string s;
            std::getline(in, s);
        }
        double currVal = -1;
        bool run = true;
        while (run) {

            std::vector<double> stamps;
            for (int i = 0; i < numbersInLine; ++i) {
                if (in >> currVal) {
                    stamps.push_back(currVal);
                } else {
                    run = false;
                    break;
                }
            }
            if (run) {
                assert(stamps.size() == 8);
                coordAndQuat.push_back(stamps);
            }
        }
    }
    std::vector<std::vector<double>> resultingTruth;
    for (int i = 0; i < timeStamps.size(); ++i) {
        for (int posInFile = 0; posInFile < coordAndQuat.size(); ++posInFile) {
            if (abs(coordAndQuat[posInFile][0] - timeStamps[i]) < abs(prevCoordinates[0] - timeStamps[i])) {
                prevCoordinates = coordAndQuat[posInFile];
            }
        }
        resultingTruth.push_back(prevCoordinates);
    }
    assert(resultingTruth.size() == timeStamps.size());
    return resultingTruth;
}

#include <limits>

int GTT::writeGroundTruth(const std::string &pathOut, const std::vector<std::vector<double>> &timeCoordinates) {
    std::ofstream out(pathOut);
    int skipN = 3;
    for (int i = 0; i < skipN; ++i) {
        out << "#\n";
    }

    for (const auto &e: timeCoordinates) {
        out.precision(std::numeric_limits<double>::max_digits10);
        out << std::setw(2 * spaceIO) << e[0];
        for (int i = 1; i < e.size(); ++i) {
            out << std::setw(2 * spaceIO) << e[i];
        }
        out << std::endl;
    }
    return 1;
}

void GTT::writeInfo(const std::vector<std::string> &rgb, const std::string &pathTimeRGB,
                    const std::string &pathToGroundTruth, const std::string &pathOut, const std::set<int> &indices) {
    std::vector<double> timeStamps = createTimestamps(rgb, pathTimeRGB, pathToGroundTruth, indices);
    std::vector<std::vector<double>> timeAndCoordinates = getGroundTruth(pathToGroundTruth, timeStamps);
    writeGroundTruth(pathOut, timeAndCoordinates);
}

void
GTT::prepareDataset(const std::string &pathToDataset, const std::string &pathOut, const std::set<int> &indicesSet, const std::string NewName = "subset") {
//    makeRotationsRelativeAndExtractImages("/home/leoneed/Desktop/coke_dataset/groundtruth.txt",
//                                          "/home/leoneed/Desktop/coke_dataset/rgb",
//                                          "/home/leoneed/Desktop/coke_dataset/depth",
//                                          "/home/leoneed/CLionProjects/GDR/test/data/coke",
//                                          "/home/leoneed/Desktop/coke_dataset/rgb.txt",
//                                          indicesSet);
    std::string pathNewOut = pathOut + "/" + NewName;
    std::string groundtruth = pathToDataset + "/groundtruth.txt";
    std::string rgb = pathToDataset + "/rgb";
    std::string depth = pathToDataset + "/depth";
    std::string timeInfo = pathToDataset + "/rgb.txt";
    makeRotationsRelativeAndExtractImages(groundtruth,
                                          rgb,
                                          depth,
                                          pathNewOut,
                                          timeInfo,
                                          indicesSet);
}