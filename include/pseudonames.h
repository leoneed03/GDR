//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_PSEUDONAMES_H
#define GDR_PSEUDONAMES_H

#include <Eigen/Eigen>

namespace gdr {

    typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
}

#endif
