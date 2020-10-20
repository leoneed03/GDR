#pragma once

#ifndef TEST_SIFTGPU_FILES_H
#define TEST_SIFTGPU_FILES_H

#include <iostream>
#include <vector>
#include <dirent.h>
#include <memory>
#include <boost/timer.hpp>

#include "assert.h"

std::vector<std::string> readRgbData(std::string pathToRGB);

#endif
