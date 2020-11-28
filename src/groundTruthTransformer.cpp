#include "../include/groundTruthTransformer.h"
#include <fstream>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

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

int GTT::makeRotationsRelativeAndExtractImages(const std::string &pathToGroundTruth, const std::string &pathToRGB, const std::string &pathToD, const std::string &pathOutDirectory, const std::set<int> indices) {
    std::ifstream in(pathToGroundTruth);
    std::string outRGB = pathOutDirectory + "/RGB";
    std::string outD = pathOutDirectory + "/D";
    namespace fs=boost::filesystem;
    fs::path path_to_remove(pathOutDirectory);
    for (fs::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it) {
        fs::remove_all(it->path());
    }
    boost::filesystem::create_directory(outD);
    boost::filesystem::create_directory(outRGB);
    std::string pathToRelativeGroundTruth = pathOutDirectory + "/groundtruth.txt";
    std::ofstream out(pathToRelativeGroundTruth);
    int numOfEmptyLines = 3;
    int numbersInLine = 8;
    std::vector<double> stamp0;
    std::vector<double> prevCoordinates = {0, 0, 0};
    bool isZero = true;
    int counter = -1;

    if (in) {
        for (int i = 0; i < 3; ++i) {
            std::string s;
            std::getline(in, s);
            out << s << std::endl;
        }
        double currVal = -1;
        for (std::string s; std::getline(in, s);) {
            ++counter;
            if (indices.find(counter) != indices.end()) {
                out << s << std::endl;
            }
        }
        makeRotationsRelative(pathToRelativeGroundTruth, pathOutDirectory + "/groundtruthR.txt");
    }
    std::vector<std::string> rgbDataR = readData(pathToRGB);
    std::vector<std::string> dDataR = readData(pathToD);
    std::vector<std::string> rgbData = readRgbData(pathToRGB);
    std::vector<std::string> dData = readRgbData(pathToD);
    assert(rgbData.size() == dData.size());
//    std::string directory = pathOutDirectory + "/RGB_D";
//    boost::filesystem::create_directory(directory);

    for (const auto& e: indices) {
        if (e >= rgbData.size()) {
            break;
        }
        std::string toRGB = outRGB + "/" + std::to_string(e) + "RGB.png";
        std::string toD = outD + "/" + std::to_string(e) + "D.png";
        boost::filesystem::copy_file(rgbData[e], toRGB);
        boost::filesystem::copy_file(dData[e], toD);

    }
//    makeRotationsRelative();
}