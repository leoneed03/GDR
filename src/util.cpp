#include "../include/util.h"

MatrixX getSomeMatrix(int height, int width) {
    return MatrixX::Random(height, width);
}

Eigen::Matrix3d getRotationMatrixDouble(const MatrixX& m) {
    Eigen::Matrix3d resMatrix;

    assert(m.col() == 3);
    assert(m.rows() == 3);


    for (int i = 0; i < 3; ++i) {
//        assert(resMatrix[i].size() == 3);
        for (int j = 0; j < 3; ++j) {
            resMatrix.row(i)[j] = m.row(i)[j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            assert(abs(resMatrix.row(i)[j] - m.row(i)[j]) < std::numeric_limits<double>::epsilon());
        }
    }
    return resMatrix;
}