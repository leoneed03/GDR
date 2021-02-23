//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "ThreadPool.h"

namespace gdr {

    ThreadPool::ThreadPool(int numOfThreads): sheduler(1/*numOfThreads*/) {

    }

    tbb::task_group &ThreadPool::getTaskGroup() {
        return tasks;
    }
}