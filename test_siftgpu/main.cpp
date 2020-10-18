#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/timer.hpp>
#include <dirent.h>

//#include <experimental/filesystem>
#include <GL/gl.h>

//#include "../include/features.h"
#include "../include/CG.h"

using namespace std;


int main(int argc, char **argv) {

    CorrespondenceGraph correspondenceGraph("../data/rgbdoffice/rgb", "../data/rgbdoffice/d");

    return 0;
}
