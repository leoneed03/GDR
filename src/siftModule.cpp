//
// Copyright (c) Microsoft Corporation and contributors. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include "../include/siftModule.h"

SiftModule::SiftModule() {
    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
}
