//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/VertexCG.h"

keypointWithDepth::keypointWithDepth(SiftGPU::SiftKeypoint newKeypoint, double newDepth,
                                     const std::vector<float> &newDescriptors) : keypoint(newKeypoint),
                                                                                 depth(newDepth),
                                                                                 descriptors(newDescriptors) {
}

void VertexCG::setRotation(const MatrixX &rotation) {
    assert(rotation.rows() == 3 && rotation.cols() == 3);
    absoluteRotationTranslation.block<3, 3>(0, 0) = rotation;
    assert(abs(rotation.col(3)[3] - 1) < std::numeric_limits<double>::epsilon);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            assert(abs(absoluteRotationTranslation.col(i)[j] - rotation.col(i)[j]) <
                   2 * std::numeric_limits<double>::epsilon);
        }
    }
}

VertexCG::VertexCG(int newIndex,
                   const std::vector<keypointWithDepth> &newKeypointsWithDepths,
                   const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
                   const std::vector<float> &newDescriptors,
                   const std::vector<double> &newDepths,
                   const std::string &newPathRGB,
                   const std::string &newPathD) : index(newIndex),
                                                  keypointsWithDepths(newKeypointsWithDepths),
                                                  keypoints(newKeypoints),
                                                  descriptors(newDescriptors),
                                                  depths(newDepths),
                                                  pathToRGBimage(newPathRGB),
                                                  pathToDimage(newPathD) {
    int transformationMatrixSize = 4;
    absoluteRotationTranslation = getSomeMatrix(transformationMatrixSize, transformationMatrixSize);
    for (int i = 0; i < transformationMatrixSize; ++i) {
        for (int j = 0; j < transformationMatrixSize; ++j) {
            absoluteRotationTranslation.col(i)[j] = 0;
        }
    }
    for (int j = 0; j < transformationMatrixSize; ++j) {
        absoluteRotationTranslation.col(j)[j] = 1;
    }
    absoluteRotationTranslation.row(transformationMatrixSize - 1)[transformationMatrixSize - 1] = 1;
}
