#include "../include/groundTruthTransformer.h"

int main() {
    GTT::makeRotationsRelative("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/groundtruth.txt", "relativeGroundTruth.txt");
    return 0;
}