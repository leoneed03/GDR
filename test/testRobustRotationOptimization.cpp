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

struct CostFunctor {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int fCeres() {
    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;
    // Build the problem.
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction *cost_function =
            new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    return 0;
}

TEST(testRotationRepresentation, Ceres) {

//    fCeres();

}

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
    gdr::Rotation3d randomRotationId(rotationMatrixId);
    std::cout << "Id rotation" << std::endl;
    std::cout << randomRotationId << std::endl;
    std::cout << "log ID is " << std::endl;
    std::cout << randomRotationId.getRotationSophus().log() << std::endl;

    Eigen::Vector3d logV = randomRotationId.getLog();
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
            lessRelRotations.emplace_back(relRot);
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


//    std::vector<gdr::poseInfo> posesInfoLess = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
//    std::vector<Eigen::Quaterniond> absoluteRotationsQuat;
//    for (const auto& pose: posesInfoLess) {
//        absoluteRotationsQuat.push_back(pose.getOrientationQuat());
//    }
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
    auto optimizedRotations = rotationOptimizer.getOptimizedOrientation();

    std::cout << "\n\nPERFORMED\n\n\n" << std::endl;

    Eigen::Matrix3d id;
    id.setIdentity();
    Eigen::Quaterniond qid(id);

    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
    std::vector<Eigen::Quaterniond> absoluteRotationsQuatFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteRotationsQuatFromGroundTruth.push_back(posesInfo[i].orientationQuat);
    }

    gdr::rotationOperations::applyRotationToAllFromLeft(absoluteRotationsQuatFromGroundTruth,
                                                        absoluteRotationsQuatFromGroundTruth[0].inverse().normalized());

    assert(absoluteRotationsQuat.size() == absoluteRotationsQuatFromGroundTruth.size());

    std::cout << "________________________________________________" << std::endl;

    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentAngleError = absoluteRotationsQuatFromGroundTruth[i].angularDistance(optimizedRotations[0].inverse().normalized() * optimizedRotations[i]);
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


TEST(testRobustRotationOptimization, fromAbsolutePoses) {

    int numPoses = 19;
    std::cout << "hey" << std::endl;


    std::string absolutePosesGroundTruth = "../../data/files/absolutePoses_19.txt";
    std::string relPoses = "../../data/files/pairWiseFirstPoseZero_19_less.txt";
    std::string absolutePoses = "../../data/files/absolutePoses_19_less.txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::readRelativePoses(relPoses);
    std::vector<gdr::poseInfo> posesRelInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<gdr::Rotation3d> orientations;
    for (int i = 0; i < posesRelInfo.size(); ++i) {
        orientations.push_back(gdr::Rotation3d(posesRelInfo[i].orientationQuat));
    }

    for (int i = 0; i < orientations.size(); ++i) {
        std::cout << i << "______" << orientations[i] << std::endl;
    }
    std::vector<gdr::rotationMeasurement> relRotMeasurements;


    for (const auto &relPose: relativePoses) {
        relRotMeasurements.push_back(relPose.getRelativeRotation());
    }

    gdr::RotationOptimizer rotationOptimizer(orientations, relRotMeasurements);


    std::cout << "PERFORM CERES" << std::endl;
    auto optimizedRotations = rotationOptimizer.getOptimizedOrientation();

    std::cout << "\n\nPERFORMED\n\n\n" << std::endl;

    std::vector<Eigen::Quaterniond> absoluteRotationsQuat;
    for (const auto& q: optimizedRotations) {
        absoluteRotationsQuat.push_back(q);
    }

    gdr::rotationOperations::applyRotationToAllFromLeft(absoluteRotationsQuat,
                                                        absoluteRotationsQuat[0].inverse().normalized());


    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGroundTruth);
    std::vector<Eigen::Quaterniond> absoluteRotationsQuatFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteRotationsQuatFromGroundTruth.push_back(posesInfo[i].orientationQuat);
    }

    gdr::rotationOperations::applyRotationToAllFromLeft(absoluteRotationsQuatFromGroundTruth,
                                                        absoluteRotationsQuatFromGroundTruth[0].inverse().normalized());

    assert(optimizedRotations.size() == absoluteRotationsQuatFromGroundTruth.size());

    std::cout << "________________________________________________" << std::endl;

    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentAngleError = absoluteRotationsQuatFromGroundTruth[i].angularDistance(absoluteRotationsQuat[i]);
        std::cout << i << ":\t" << currentAngleError << std::endl;
        sumErrors += currentAngleError;
        sumErrorsSquared += pow(currentAngleError, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    int counter = 0;
    for (const auto& optimizedOrientation: optimizedRotations) {
        Eigen::Quaterniond q = optimizedOrientation;
        std::cout << counter << "<optimized>: " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ' '<< std::endl;
    }
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;



    ASSERT_LE(meanError, 1e-2);
}


TEST(testRobustRotationOptimization, fromAbsolutePosesNoCeres) {

    int numPoses = 19;
    std::cout << "hey" << std::endl;


    std::string absolutePosesGroundTruth = "../../data/files/absolutePoses_19.txt";
    std::string relPoses = "../../data/files/pairWiseFirstPoseZero_19_less.txt";
    std::string absolutePoses = "../../data/files/absolutePoses_19_less.txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::readRelativePoses(relPoses);
    std::vector<gdr::poseInfo> absolutePosesWithOutliers = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Eigen::Quaterniond> orientationsQuat;
    for (int i = 0; i < absolutePosesWithOutliers.size(); ++i) {
        orientationsQuat.push_back(absolutePosesWithOutliers[i].orientationQuat);
    }

    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGroundTruth);
    std::vector<Eigen::Quaterniond> absoluteRotationsQuatFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteRotationsQuatFromGroundTruth.push_back(posesInfo[i].orientationQuat);
    }

    gdr::rotationOperations::applyRotationToAllFromLeft(absoluteRotationsQuatFromGroundTruth,
                                                        absoluteRotationsQuatFromGroundTruth[0].inverse().normalized());

    assert(orientationsQuat.size() == absoluteRotationsQuatFromGroundTruth.size());

    std::cout << "________________________________________________" << std::endl;

    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentAngleError = absoluteRotationsQuatFromGroundTruth[i].angularDistance(orientationsQuat[i]);
        std::cout << i << ":\t" << currentAngleError << std::endl;
        sumErrors += currentAngleError;
        sumErrorsSquared += pow(currentAngleError, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;



    ASSERT_GE(meanError, 1e-1);
}



int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}