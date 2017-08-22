//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_ALECTRYONMATH_HPP
#define ALECTRYONMATH_ALECTRYONMATH_HPP

#include <exception>
#include <stdexcept>
#include <functional>
#include <cmath>

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
using RowVector3 = Eigen::Matrix<T, 1, 3>;

template<class T>
using RowVector4 = Eigen::Matrix<T, 1, 4>;

template<class T>
using RowVectorX = Eigen::Matrix<T, 1, Eigen::Dynamic>;

template<class T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;

template<class T>
using Matrix4 = Eigen::Matrix<T, 4, 4>;

template<class T>
using Matrix5 = Eigen::Matrix<T, 5, 5>;

template<class T>
using Matrix3X = Eigen::Matrix<T, 3, Eigen::Dynamic>;

template<class T>
using Matrix4X = Eigen::Matrix<T, 4, Eigen::Dynamic>;

template<class T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template<class T>
using ArrayX = Eigen::Array<T, Eigen::Dynamic, 1>;

}


// Function Definitions
namespace Alectryon {
namespace Math {
// TODO: instead of using eps, use relative floating point error (http://floating-point-gui.de/errors/comparison/)
const float eps = 1e-12;

template<class T>
constexpr T Pi() {
    return M_PI;
}

template<class T>
constexpr T TwoPi() {
    return 2.0 * Pi<T>();
}

template<class T>
constexpr T HalfPi() {
    return 0.5 * Pi<T>();
}

/**
 * @brief Coefficient-wise atan2
 *
 * @tparam T
 * @param y
 * @param x
 * @return
 */
template<class T>
Eigen::MatrixX<T> atan2(const Eigen::Ref<const Eigen::MatrixX<T>> &y, const Eigen::Ref<const Eigen::MatrixX<T>> &x);


template<class T>
T threshold(T val, T min, T max);

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
Eigen::Matrix3<T> cross_prod_mat(const Eigen::Vector3<T> &vec);

/**
* @brief Normalizes a matrix so that the 2-norms of each column are 1
* @tparam T
* @param dst
* @param src
*/
template<class T>
void normalize_mat(const Eigen::Ref<const Eigen::MatrixX<T>> &src, Eigen::Ref<Eigen::MatrixX<T>> dst);

/**
* @brief Normalizes a matrix so that the 2-norms of each column are 1
* @tparam T
* @param src
* @return normalized matrix
*/
template<class T>
Eigen::MatrixX<T> normalize_mat(const Eigen::Ref<const Eigen::MatrixX<T>> &src);

/**
* @brief Normalizes a matrix so that the 2-norms of each column are 1 inplace
* @tparam T
* @param src
*/
template<class T>
void normalize_mat_inplace(Eigen::Ref<Eigen::MatrixX<T>> mat);

}
}

// Implementation
namespace Alectryon {
namespace Math {

template<class T>
Eigen::MatrixX<T> atan2(const Eigen::Ref<const Eigen::MatrixX<T>> &y, const Eigen::Ref<const Eigen::MatrixX<T>> &x) {
    if (y.rows() != x.rows() or y.cols() != x.cols()) {
        throw std::runtime_error("y and x must be same size");
    }

    auto atan2_helper = [](const T &y, const T &x) {return std::atan2(y, x);};
    Eigen::MatrixX<T> dst = y.binaryExpr(x, atan2_helper);
    return dst;
}

template<class T>
T threshold(T val, T min, T max) {
    if (min > max) {
        throw std::invalid_argument("min can't be greater than max");
    }

    if (val > max) val = max;
    else if (val < min) val = min;
    return val;
}

template<class T>
Eigen::Matrix3<T> cross_prod_mat(const Eigen::Vector3<T> &vec) {
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

template<class T>
void normalize_mat(const Eigen::Ref<const Eigen::MatrixX<T>> &src, Eigen::Ref<Eigen::MatrixX<T>> dst) {
    Eigen::VectorX<T> norms = src.colwise().norm();
    dst = src.array().rowwise() / norms.transpose().array();
}

template<class T>
Eigen::MatrixX<T> normalize_mat(const Eigen::Ref<const Eigen::MatrixX<T>> &src) {
    Eigen::MatrixX<T> dst(src.rows(), src.cols());
    normalize_mat<T>(src, dst);
    return dst;
}

template<class T>
void normalize_mat_inplace(Eigen::Ref<Eigen::MatrixX<T>> mat) {
    normalize_mat<T>(mat, mat);
}



}
}

#endif //ALECTRYONMATH_ALECTRYONMATH_HPP
