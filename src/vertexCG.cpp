#include "../include/vertexCG.h"

vertexCG::vertexCG(int newIndex,
                   const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
                   const std::vector<float> &newDescriptors,
                   const std::vector<int> &newDepths,
                   const std::string &newPathRGB,
                   const std::string &newPathD) : index(newIndex),
                                                  keypoints(newKeypoints),
                                                  descriptors(newDescriptors),
                                                  depths(newDepths),
                                                  pathToRGBimage(newPathRGB),
                                                  pathToDimage(newPathD) {
}
