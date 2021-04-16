//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "boost/filesystem.hpp"

#include "poseGraph/VertexPose.h"

namespace gdr {

    const std::vector<KeyPoint2DAndDepth> &VertexPose::getKeyPoints() const {
        return keypoints;
    }

    const std::vector<float> &VertexPose::getDescriptors() const {
        return descriptors;
    }

    const std::vector<double> &VertexPose::getDepths() const {
        return depths;
    }

    std::string VertexPose::getPathRGBImage() const {
        return pathToRGBimage;
    }

    std::string VertexPose::getFilenameRGBImage() const {
        boost::filesystem::path path(getPathRGBImage());

        return path.filename().string();
    }

    std::string VertexPose::getPathDImage() const {
        return pathToDimage;
    }

    std::string VertexPose::getFilenameDImage() const {
        boost::filesystem::path path(getPathDImage());

        return path.filename().string();
    }

    const CameraRGBD &VertexPose::getCamera() const {
        return cameraRgbd;
    }

    int VertexPose::getIndex() const {
        return index;
    }


    void VertexPose::setRotation(const Sophus::SO3d &rotation) {
        absolutePose.setRotation(rotation);
    }

    void VertexPose::setRotation(const SO3 &rotation) {
        absolutePose.setRotation(rotation);
    }

    void VertexPose::setRotation(const Eigen::Matrix3d &rotation) {
        Eigen::Quaterniond quatRotation(rotation);
        absolutePose.setRotation(Sophus::SO3d(quatRotation.normalized()));
    }

    void VertexPose::setRotation(const Eigen::Quaterniond &rotationQuatd) {
        absolutePose.setRotation(Sophus::SO3d(rotationQuatd.normalized()));
    }

    void VertexPose::setTranslation(const Eigen::Vector3d &translationToSet) {
        absolutePose.setTranslation(translationToSet);
        double error = (absolutePose.getTranslation() - translationToSet).norm();
        int coeffForEps = 100;
        assert(error < coeffForEps * std::numeric_limits<double>::epsilon());
    }

    void VertexPose::setRotationTranslation(const Eigen::Matrix4d &eigenRt) {
        absolutePose = SE3(Sophus::SE3d::fitToSE3(eigenRt));
    }

    void VertexPose::setRotationTranslation(const Sophus::SE3d &sophusRt) {
        absolutePose = SE3(sophusRt);
    }

    Eigen::Matrix4d VertexPose::getEigenMatrixAbsolutePose4d() const {
        return absolutePose.getSE3().matrix();
    }

    const SE3 &VertexPose::getAbsolutePoseSE3() const {
        return absolutePose;
    }


    const std::vector<KeyPoint2DAndDepth> &VertexPose::getKeyPoints2D() const {
        return keypoints;
    }

    Eigen::Quaterniond VertexPose::getRotationQuat() const {
        return absolutePose.getSE3().unit_quaternion();
    }


    Sophus::SE3d VertexPose::getAbsolutePoseSophus() const {
        return absolutePose.getSE3();
    }

    VertexPose::VertexPose(int newIndex,
                       const CameraRGBD &newCameraRgbd,
                       const keyPointsDepthDescriptor &keyPointsDepthDescriptor,
                       const std::string &newPathRGB,
                       const std::string &newPathD,
                       double timestampToSet) : index(newIndex),
                                                initialIndex(newIndex),
                                                cameraRgbd(newCameraRgbd),
                                                keypoints(keyPointsDepthDescriptor.getKeyPointsKnownDepth()),
                                                descriptors(keyPointsDepthDescriptor.getDescriptorsKnownDepth()),
                                                depths(keyPointsDepthDescriptor.getDepths()),
                                                pathToRGBimage(newPathRGB),
                                                pathToDimage(newPathD),
                                                timestamp(timestampToSet) {}

    void VertexPose::setIndex(int newIndex) {
        index = newIndex;
    }

    void VertexPose::setAbsolutePoseSE3(const SE3 &absolutePoseToSet) {
        absolutePose = absolutePoseToSet;
    }

    void VertexPose::setCamera(const CameraRGBD &camera) {
        cameraRgbd = camera;
    }

    int VertexPose::getInitialIndex() const {
        return initialIndex;
    }

    int VertexPose::getKeyPointsNumber() const {
        return keypoints.size();
    }

    const KeyPoint2DAndDepth &VertexPose::getKeyPoint(int keyPointIndex) const {
        assert(keyPointIndexIsValid(keyPointIndex));

        return keypoints[keyPointIndex];
    }

    bool VertexPose::keyPointIndexIsValid(int keyPointIndex) const {
        assert(keyPointIndex >= 0);
        assert(keyPointIndex < keypoints.size());
        int keyPointsNum = keypoints.size();
        assert(keyPointsNum == depths.size());
        assert(keyPointsNum * 128 == descriptors.size());

        return true;
    }

    double VertexPose::getKeyPointDepth(int keyPointIndex) const {
        assert(keyPointIndexIsValid(keyPointIndex));

        return depths[keyPointIndex];
    }

    double VertexPose::getTimestamp() const {
        return timestamp;
    }
}
