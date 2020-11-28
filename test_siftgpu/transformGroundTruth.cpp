#include "../include/groundTruthTransformer.h"

int main() {
    std::set<int> indicesSet;
    for (int i = 0; i < 25; i += 3) {
        indicesSet.insert(i);
    }
    GTT::makeRotationsRelativeAndExtractImages("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/groundtruth.txt",
                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/rgb_original",
                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/depth_original",
                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/extracted",
                                               indicesSet);

//    GTT::makeRotationsRelative("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/groundtruth.txt", "relativeGroundTruth.txt");
    return 0;
}