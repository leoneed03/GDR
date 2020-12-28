//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "VertexCG.h"

namespace gdr {

    void VertexCG::setRotation(const Eigen::Matrix3d &rotation) {

        absoluteRotationTranslation.block<3, 3>(0, 0) = rotation;
    }

    VertexCG::VertexCG(int newIndex,
                       const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
                       const std::vector<float> &newDescriptors,
                       const std::vector<double> &newDepths,
                       const std::string &newPathRGB,
                       const std::string &newPathD) : index(newIndex),
                                                      keypoints(newKeypoints),
                                                      descriptors(newDescriptors),
                                                      depths(newDepths),
                                                      pathToRGBimage(newPathRGB),
                                                      pathToDimage(newPathD) {
        absoluteRotationTranslation.setIdentity();
    }
}
