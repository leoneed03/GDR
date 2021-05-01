//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/ThreadPoolTBB.h"

namespace gdr {

    ThreadPoolTBB::ThreadPoolTBB(int numOfThreads) : sheduler(numOfThreads) {}
}