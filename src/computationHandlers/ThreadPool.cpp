//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/ThreadPool.h"

namespace gdr {

    ThreadPool::ThreadPool(int numOfThreads): sheduler(numOfThreads) {

    }

    tbb::task_group &ThreadPool::getTaskGroup() {
        return tasks;
    }
}