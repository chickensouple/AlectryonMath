//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_ALECTRYONMATH_HPP
#define ALECTRYONMATH_ALECTRYONMATH_HPP

#include <exception>
#include <stdexcept>

#include <Eigen/Dense>

// convenience types for eigen
namespace Eigen {

template<class T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template<class T>
using Vector4 = Eigen::Matrix<T, 4, 1>;

template<class T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template<class T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;

template<class T>
using Matrix4 = Eigen::Matrix<T, 4, 4>;

template<class T>
using Matrix3X = Eigen::Matrix<T, 3, Eigen::Dynamic>;

template<class T>
using Matrix4X = Eigen::Matrix<T, 4, Eigen::Dynamic>;

template<class T>
using MatrixXX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

}


// Function Definitions
namespace Alectryon {
namespace Math {
// TODO: instead of using eps, use relative floating point error (http://floating-point-gui.de/errors/comparison/)
const float eps = 1e-12;

template<class T>
T threshold(T val, T min, T max);


}
}

// Implementation
namespace Alectryon {
namespace Math {

template<class T>
T threshold(T val, T min, T max) {
    if (min > max) {
        throw std::invalid_argument("min can't be greater than max");
    }

    if (val > max) val = max;
    else if (val < min) val = min;
    return val;
}

/**
 * @brief Makes the matrix representation of a cross product with the given vector
 * @details Makes a matrix A out of a vector v such that for
 * any vector x, A*x = cross(v, x)
 *
 * @tparam T
 * @param vec
 * @return skew symmetric matrix
 */
template<class T>
Eigen::Matrix3<T> cross_prod_matrix(const Eigen::Vector3<T> &vec) {
    Eigen::Matrix3<T> mat;
    mat(0, 0) = 0;
    mat(1, 1) = 0;
    mat(2, 2) = 0;
    mat(1, 0) = vec[2];
    mat(0, 1) = -vec[2];
    mat(0, 2) = vec[1];
    mat(2, 0) = -vec[1];
    mat(2, 1) = vec[0];
    mat(1, 2) = -vec[0];
    return mat;
};


}
}

#endif //ALECTRYONMATH_ALECTRYONMATH_HPP
