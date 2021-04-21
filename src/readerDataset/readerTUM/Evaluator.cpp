//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "readerDataset/readerTUM/Evaluator.h"
#include "readerDataset/readerTUM/ReaderTum.h"
#include "readerDataset/readerTUM/ClosestMatchFinder.h"

#include <fstream>
#include <algorithm>

namespace gdr {

    Evaluator::Evaluator(const std::string &groundtruthFile) {

        std::vector<PoseFullInfo> posesReadFromFile = ReaderTUM::getPoseInfoTimeTranslationOrientation(groundtruthFile);
        initFromPosesVector(posesReadFromFile);
    }

    Evaluator::Evaluator(const std::vector<PoseFullInfo> &poses) {
        initFromPosesVector(poses);
    }

    void Evaluator::initFromPosesVector(const std::vector<PoseFullInfo> &poses) {
        posesGroundTruth = poses;
        std::sort(posesGroundTruth.begin(), posesGroundTruth.end(),
                  [](const PoseFullInfo &lhs, const PoseFullInfo &rhs) {
                      return lhs.getTimestamp() < rhs.getTimestamp();
                  });

        for (const auto &pose: posesGroundTruth) {
            poseGroundTruthByTime.insert(std::make_pair(pose.getTimestamp(), pose));
        }
    }


    ErrorRotationTranslation Evaluator::evaluateTrajectory(const std::vector<PoseFullInfo> &trajectory,
                                                           int indexFixed,
                                                           bool alignWithUmeyama,
                                                           double maxTimeDiff,
                                                           const std::string &pathOutAlignedGroundTruth,
                                                           const std::string &pathOutAlignedTrajectory) const {

        auto setOfPosesFromGroundTruth = poseGroundTruthByTime;

        if (setOfPosesFromGroundTruth.empty()) {
            std::cout << "Provided ground thruth file is empty" << std::endl;
            return ErrorRotationTranslation(
                    {"", "", 0, 0, 0, 0, 0, 0},
                    {"", "", 0, 0, 0, 0, 0, 0});
        }

        double sumErrorL2 = 0;
        double sumErrorRadian = 0;
        double sumErrorL2Squared = 0;
        double sumErrorRadianSquared = 0;

        std::vector<double> errorsL2;
        std::vector<double> errorsRadians;
        std::vector<double> errorsL2Squared;
        std::vector<double> errorsRadiansSquared;

        auto closestMatchToFixedZero = ClosestMatchFinder::findClosestKeyMatch<double, PoseFullInfo>(
                setOfPosesFromGroundTruth, trajectory[indexFixed].getTimestamp()
        );
        assert(closestMatchToFixedZero != setOfPosesFromGroundTruth.end());

        auto fixedPoseGroundTruth(closestMatchToFixedZero->second.getSophusPose());

        SE3 zeroPoseGt;
        SE3 zeroPoseTrajectory;

        int iteration = 0;

        std::vector<PoseFullInfo> posesMatchedGroundTruth;
        std::vector<PoseFullInfo> posesMatchedTrajectory;


        for (const auto &poseTrajectory: trajectory) {

            auto closestMatch = ClosestMatchFinder::findClosestKeyMatch<double, PoseFullInfo>(
                    setOfPosesFromGroundTruth, poseTrajectory.getTimestamp()
            );
            assert(closestMatch != setOfPosesFromGroundTruth.end());


            double foundTimestamp = closestMatch->second.getTimestamp();
            double poseTimestamp = poseTrajectory.getTimestamp();

            if (std::abs(foundTimestamp - poseTimestamp) > maxTimeDiff) {
                std::cout.precision(std::numeric_limits<double>::max_digits10);
                std::cout << "found timestamp " << foundTimestamp
                          << " while looking for " << poseTimestamp << std::endl;
                continue;
            }

            posesMatchedGroundTruth.emplace_back(closestMatch->second);
            posesMatchedTrajectory.emplace_back(poseTrajectory);
        }

        SE3 transformationAlignmentUmeyama;
        {

            // can also check ATE with this alignment:
            Eigen::Matrix3Xd pointsGroundTruth(3, posesMatchedTrajectory.size());
            Eigen::Matrix3Xd pointsTrajectory(3, posesMatchedGroundTruth.size());

            for (int point = 0; point < posesMatchedGroundTruth.size(); ++point) {
                pointsGroundTruth.col(point) = posesMatchedGroundTruth[point].getTranslation();
                pointsTrajectory.col(point) = posesMatchedTrajectory[point].getTranslation();
            }

            //align trajectories using umeyama + fit found (cR+t)-transformation to R+t [SE3]
            transformationAlignmentUmeyama = SE3(Eigen::umeyama(pointsGroundTruth, pointsTrajectory));
        }

        auto posesMatchedGroundTruthAlignedUmeyama(posesMatchedGroundTruth);


        for (int poseGroundTruthIndex = 0; poseGroundTruthIndex < posesMatchedGroundTruth.size(); ++poseGroundTruthIndex) {

            {
                auto &pose = posesMatchedGroundTruth[poseGroundTruthIndex];
                SE3 alignedGroundTruthPoseZeroPose(fixedPoseGroundTruth.inverse() * pose.getSophusPose());
                pose = PoseFullInfo(pose.getTimestamp(), alignedGroundTruthPoseZeroPose);
            }

            {
                auto &poseUmeyama = posesMatchedGroundTruthAlignedUmeyama[poseGroundTruthIndex];
                SE3 alignedGroundTruthPoseUmeyama(transformationAlignmentUmeyama * poseUmeyama.getSophusPose());
                poseUmeyama = PoseFullInfo(poseUmeyama.getTimestamp(), alignedGroundTruthPoseUmeyama);
            }
        }

        // compare with umeyama aligned if the flag is true
        if (alignWithUmeyama) {
            std::swap(posesMatchedGroundTruth, posesMatchedGroundTruthAlignedUmeyama);
        }

        if (!pathOutAlignedGroundTruth.empty()) {
            std::ofstream outGroundTruthAligned(pathOutAlignedGroundTruth);

            for (const auto &poseAligned: posesMatchedGroundTruth) {
                outGroundTruthAligned << poseAligned << std::endl;
            }
        }

        for (auto &pose: posesMatchedTrajectory) {
            if (alignWithUmeyama) {
                pose = PoseFullInfo(pose.getTimestamp(), SE3(pose.getSophusPose()));
            } else {
                pose = PoseFullInfo(pose.getTimestamp(), SE3(trajectory[indexFixed].getSophusPose().inverse()
                                            * pose.getSophusPose()));
            }
        }

        if (!pathOutAlignedTrajectory.empty()) {
            std::ofstream outTrajectoryAligned(pathOutAlignedTrajectory);

            for (const auto &poseAligned: posesMatchedTrajectory) {
                outTrajectoryAligned << poseAligned << std::endl;
            }
        }
        for (int pose = 0; pose < posesMatchedGroundTruth.size(); ++pose) {

            //align both estimated trajectory, groundtruth is already aligned
            SE3 alignedGroundTruthPose(posesMatchedGroundTruth[pose].getSophusPose());

            SE3 alignedTrajectoryPose;

            if (alignWithUmeyama) {
                alignedTrajectoryPose = SE3(posesMatchedTrajectory[pose].getSophusPose());
            } else {
                alignedTrajectoryPose = SE3(trajectory[indexFixed].getSophusPose().inverse()
                                         * posesMatchedTrajectory[pose].getSophusPose());
            }

            std::pair<double, double> errorRotationTranslation =
                    alignedGroundTruthPose.getRotationTranslationErrors(alignedTrajectoryPose);

            double errorL2 = errorRotationTranslation.second;
            double errorRadian = errorRotationTranslation.first;

            errorsL2.emplace_back(errorL2);
            errorsRadians.emplace_back(errorRadian);

            errorsL2Squared.emplace_back(errorL2 * errorL2);
            errorsRadiansSquared.emplace_back(errorRadian * errorRadian);

            sumErrorL2 += errorL2;
            sumErrorRadian += errorRadian;

            sumErrorL2Squared += errorL2 * errorL2;
            sumErrorRadianSquared += errorRadian * errorRadian;
        }

        sort(errorsL2);
        sort(errorsRadians);
        sort(errorsL2Squared);
        sort(errorsRadiansSquared);

        assert(errorsRadians.size() == errorsRadiansSquared.size());
        assert(errorsRadians.size() == errorsL2.size());
        assert(errorsL2.size() == errorsL2Squared.size());
        assert(!errorsL2.empty());


        int successfulMatchings = errorsL2.size();

        double meanL2 = sumErrorL2 / successfulMatchings;
        double meanRadian = sumErrorRadian / successfulMatchings;
        double meanL2Squared = sumErrorL2Squared / successfulMatchings;
        double meanRadianSquared = sumErrorRadianSquared / successfulMatchings;
        double medianL2 = median(errorsL2);
        double medianRadian = median(errorsRadians);

        double rmseL2 = std::sqrt(meanL2Squared);
        double rmseRadian = std::sqrt(meanRadianSquared);

        double stdL2 = std::sqrt(meanL2Squared - std::pow(meanL2, 2));
        double stdRadian = std::sqrt(meanRadianSquared - std::pow(meanRadian, 2));

        ErrorInformation errorInformationRadian("absolute_rotation_error", "",
                                                rmseRadian,
                                                meanRadian,
                                                medianRadian,
                                                stdRadian,
                                                errorsRadians[0],
                                                errorsRadians[successfulMatchings - 1]);

        ErrorInformation errorInformationL2("absolute_translational_error", "m",
                                            rmseL2,
                                            meanL2,
                                            medianL2,
                                            stdL2,
                                            errorsL2[0],
                                            errorsL2[successfulMatchings - 1]);

        ErrorRotationTranslation errorsRotTransl(errorInformationRadian,
                                                 errorInformationL2);

        errorsRotTransl.numberOfPosesEvaluated = successfulMatchings;
        errorsRotTransl.numberOfPosesGroundTruth = posesGroundTruth.size();
        errorsRotTransl.numberOfPosesTrajectory = trajectory.size();

        return errorsRotTransl;
    }

    void Evaluator::sort(std::vector<double> &values) {
        std::sort(values.begin(), values.end());
    }

    double Evaluator::median(const std::vector<double> &valuesNotSorted) {

        auto values(valuesNotSorted);
        sort(values);
        assert(std::is_sorted(values.begin(), values.end()));

        assert(!values.empty());

        int sizeVec = values.size();

        if (sizeVec % 2 == 0) {
            return 0.5 * (values[sizeVec / 2] + values[sizeVec / 2 + 1]);
        } else {
            return values[sizeVec / 2];
        }
    }


    std::ostream &operator<<(std::ostream &os, const ErrorInformation &info) {

        os << info.nameError << ".rmse   " << info.RMSE << " " << info.typeOfMeasurement << std::endl;
        os << info.nameError << ".mean   " << info.MEAN << " " << info.typeOfMeasurement << std::endl;
        os << info.nameError << ".median " << info.MEDIAN << " " << info.typeOfMeasurement << std::endl;
        os << info.nameError << ".std    " << info.STD << " " << info.typeOfMeasurement << std::endl;
        os << info.nameError << ".min    " << info.MINERR << " " << info.typeOfMeasurement << std::endl;
        os << info.nameError << ".max    " << info.MAXERR << " " << info.typeOfMeasurement << std::endl;

        return os;
    }

    std::ostream &operator<<(std::ostream &os, const ErrorRotationTranslation &informationErrors) {

        os << informationErrors.rotationError
                  << "------------------------------------------------------------" << std::endl;
        os << informationErrors.translationError << std::endl;

        os << "Compared with groundtruth: " << informationErrors.numberOfPosesEvaluated << "/"
                  << informationErrors.numberOfPosesTrajectory;
        return os;
    }
}
