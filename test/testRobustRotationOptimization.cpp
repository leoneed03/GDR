//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "Rotation3d.h"

#include "groundTruthTransformer.h"
#include "rotationAveraging.h"
#include "Rotation3d.h"
#include "quaternions.h"

#include "RotationOptimizationRobust.h"

#define epsilonD (std::numeric_limits<double>::epsilon())


#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


TEST(testRotationRepresentation, testConstructor) {


    Eigen::Matrix3d rotationMatrix;
    std::vector<double> angles = {30, 50, -87};

    rotationMatrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ())
                     * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());

    gdr::Rotation3d randomRotation(rotationMatrix);

    std::cout << "\nRandom rotation:" << std::endl;
    std::cout << randomRotation << std::endl;
    std::cout << "log is " << std::endl;
    std::cout << randomRotation.getRotationSophus().log() << std::endl;


    Eigen::Matrix3d rotationMatrixId;
    rotationMatrixId.setIdentity();
    gdr::Rotation3d rotationId(rotationMatrixId);
    std::cout << "Id rotation" << std::endl;
    std::cout << rotationId << std::endl;
    std::cout << "log ID is " << std::endl;
    std::cout << rotationId.getRotationSophus().log() << std::endl;

    Eigen::Vector3d logV = rotationId.getLog();
    std::cout << "zero?" << std::endl;
    std::cout << logV << std::endl;

    Eigen::Vector3d logNotZero = randomRotation.getLog();


    ASSERT_LE(logV.norm(), epsilonD);
    ASSERT_GE(logNotZero.norm(), epsilonD);

}


TEST(testRobustRotationOptimization, errorShouldBeZeroFirstPoseNotZero) {

    int numPoses = 19;
    std::cout << "hey" << std::endl;
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::string relativeRotations = "pairWiseFirstPoseZero_19.txt";

    std::string relativeRotationsLess = "pairWiseFirstPoseZero_19_less.txt";
    std::string absoluteRotations = "absoluteRotationsTestShanonAveraging_19_less.txt";
    std::vector<gdr::relativePose> relativePosesVector = gdr::GTT::extractAllRelativeTransformationPairwise(
            absolutePoses,
            relativeRotations,
            "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000");
    std::vector<gdr::relativePose> lessRelRotations;

    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
    std::vector<Sophus::SE3d> absolutePosesFromGroundTruth;
    Sophus::SE3d poseZeroGT;
    poseZeroGT.setQuaternion(posesInfo[0].getOrientationQuat().normalized());
    poseZeroGT.translation() = posesInfo[0].getTranslation();

    for (int i = 0; i < posesInfo.size(); ++i) {
        Sophus::SE3d absolutePoseGT;
        absolutePoseGT.setQuaternion(posesInfo[i].getOrientationQuat().normalized());
        absolutePoseGT.translation() = posesInfo[i].getTranslation();
        absolutePosesFromGroundTruth.push_back(poseZeroGT.inverse() * absolutePoseGT);
    }

    for (int i = 0; i < relativePosesVector.size(); ++i) {

        const auto &relRot = relativePosesVector[i];

        double angleZ = (10 + i * 5) % 90;
        double angleY = (5 + i * 3) % 90;
        double angleX = (-10 - i * 10) % 90;
        std::vector<double> angles = {angleZ, angleY, angleX};

        Eigen::Quaterniond rotationMatrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ())
                                            * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())
                                            * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());

        gdr::Rotation3d randomRotation(rotationMatrix);


        int indexTo = relRot.getIndexToDestination();
        int indexFrom = relRot.getIndexFromToBeTransformed();

        if (indexTo == indexFrom + 1 ||
            indexTo == indexFrom + 2 ||
            indexTo == indexFrom + 3) {
            lessRelRotations.emplace_back(gdr::relativePose(gdr::rotationMeasurement((absolutePosesFromGroundTruth[indexFrom].inverse() * absolutePosesFromGroundTruth[indexTo]).unit_quaternion(), indexFrom, indexTo),
                                                            gdr::translationMeasurement(Eigen::Vector3d(), indexFrom, indexTo)));
        }

        if (indexTo == 4 || indexTo == 6 || indexTo == 10) {
            if (indexTo == indexFrom + 4) {
                gdr::relativePose relPoseOutlier(gdr::rotationMeasurement(rotationMatrix, indexFrom, indexTo),
                                                 relRot.getRelativeTranslation());
                lessRelRotations.emplace_back(relPoseOutlier);
            }
        }

    }
    gdr::GTT::printRelativePosesFile(lessRelRotations, relativeRotationsLess, numPoses);
    std::vector<Eigen::Quaterniond> absoluteRotationsQuat = gdr::rotationAverager::shanonAveraging(
            relativeRotationsLess,
            absoluteRotations);
    std::vector<gdr::Rotation3d> orientations;
    std::vector<gdr::rotationMeasurement> relRotMeasurements;

    for (const auto &quat: absoluteRotationsQuat) {
        orientations.push_back(gdr::Rotation3d(quat));
    }
    for (const auto &relPose: lessRelRotations) {
        relRotMeasurements.push_back(relPose.getRelativeRotation());
    }

    gdr::RotationOptimizer rotationOptimizer(orientations, relRotMeasurements);


    std::cout << "PERFORM CERES" << std::endl;
    std::vector<Eigen::Quaterniond> optimizedRotations;
    optimizedRotations = rotationOptimizer.getOptimizedOrientation();

    std::cout << "\n\nPERFORMED\n\n\n" << std::endl;

    std::cout << "________________________________________________" << std::endl;

    double sumErrors = 0;
    double sumErrorsSquared = 0;

    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentAngleError = absolutePosesFromGroundTruth[i].unit_quaternion().angularDistance(optimizedRotations[0].inverse().normalized() * optimizedRotations[i]);
        std::cout << i << ":\t" << currentAngleError << std::endl;
        sumErrors += currentAngleError;
        sumErrorsSquared += pow(currentAngleError, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;

    ASSERT_LE(meanError, 1e-2);
}


int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}