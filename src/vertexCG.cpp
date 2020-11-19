#include "../include/vertexCG.h"

keypointWithDepth::keypointWithDepth(SiftGPU::SiftKeypoint newKeypoint, double newDepth,
                                     const std::vector<float> &newDescriptors) : keypoint(newKeypoint),
                                                                           depth(newDepth),
                                                                           descriptors(newDescriptors) {
}

vertexCG::vertexCG(int newIndex,
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
}
