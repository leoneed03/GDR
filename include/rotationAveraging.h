//
// Copyright (c) Microsoft Corporation and contributors. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_ROTATIONAVERAGING_H
#define TEST_SIFTGPU_ROTATIONAVERAGING_H

#include <string>

struct rotationAverager {
    static int shanonAveraging(const std::string &pathToRelativeRotations, const std::string &pathOut);
};

#endif
