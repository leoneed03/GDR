//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "util.h"

gdr::MatrixX gdr::getSomeMatrix(int height, int width) {
    return MatrixX::Random(height, width);
}
