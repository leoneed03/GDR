#include "../include/groundTruthTransformer.h"
#include <fstream>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU> // required for MatrixBase::determinant
#include <Eigen/SVD> // required for SVD
#include <iomanip>

void putAligned(std::ofstream& of, const std::vector<double>& val) {
    for (const auto& vals: val) {
        (of << std::setw(12) << vals << ' ');
    }
}

int GTT::makeRotationsRelative(const std::string &pathToGroundTruth, const std::string &pathToRelativeGroundTruth) {
    std::ifstream in(pathToGroundTruth);
    std::ofstream out(pathToRelativeGroundTruth);
    int numOfEmptyLines = 3;
    int numbersInLine = 8;
    std::vector<double> stamp0;
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
            std::vector<double> toStream = {stamps[0] - stamp0[0], stamps[1], stamps[2], stamps[3], qRelative.x(), qRelative.y(), qRelative.z(), qRelative.w()};
            putAligned(out, toStream);
            out << std::endl;
            //out << stamps[0] - stamp0[0] << ' ' << stamps[1] << ' ' << stamps[2] << ' ' << stamps[3] << ' ' << qRelative.x() << ' ' << qRelative.y() << ' ' << qRelative.z() << ' ' << qRelative.w() << std::endl;

        }

    }
}