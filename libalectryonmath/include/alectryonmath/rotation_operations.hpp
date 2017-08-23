//
// Created by clark on 4/15/17.
//

#ifndef ALECTYONMATH_ROTATIONOPERATIONS_HPP
#define ALECTYONMATH_ROTATIONOPERATIONS_HPP

#include <cmath>
#include <alectryonmath/alectryon_math.hpp>

// Function Definitions
namespace Alectryon {
namespace Transform {
// Rotation Matrix Operations
template<class T>
Eigen::Matrix2<T> rot2d(T angle);

template<class T>
Eigen::Matrix3<T> rotx(T angle);

template<class T>
Eigen::Matrix3<T> roty(T angle);

template<class T>
Eigen::Matrix3<T> rotz(T angle);

// Quaternion Operations
/**
 * @brief Creates a unit quaternion
 */
template<class T>
Eigen::Vector4<T> quat_unit();

/**
 * @brief Quaternion Multiplication. Calculates quats_res = quats1 * quats2
 * @details quats1, quats2, and quats_res must all be the same size
 * The size must be 4 by N, where each column is a separate quaternion
 * the quaternions are multiplied such that quats_res[:, i] = quats1[:, i] * quats2[:, i]
 *
 * @param quats1
 * @param quats2
 * @param quats_res
 */
template<class T>
void quat_multiply(const Eigen::Ref<const Eigen::Matrix4X<T>> &quats1,
                   const Eigen::Ref<const Eigen::Matrix4X<T>> &quats2,
                   Eigen::Ref<Eigen::Matrix4X<T>> quats_res);

/**
 * @brief Quaternion Multiplication
 *
 * @tparam T
 * @param quats1
 * @param quats2
 * @return results of multiplication
 */
template<class T>
Eigen::Matrix4X<T> quat_multiply(const Eigen::Ref<const Eigen::Matrix4X<T>> &quats1,
                                 const Eigen::Ref<const Eigen::Matrix4X<T>> &quats2);

template<class T>
Eigen::Matrix4X<T> quat_inv(const Eigen::Ref<const Eigen::Matrix4X<T>> &quat);

template<class T>
void quat_inv_inplace(Eigen::Ref<Eigen::Matrix4X<T>> quat);

}
}

// Implementation
namespace Alectryon {
namespace Transform {
// Rotation Matrix Operations
template<class T>
Eigen::Matrix2<T> rot2d(T angle) {
    Eigen::Matrix2<T> rot;
    T cos_ang = std::cos(angle);
    T sin_ang = std::sin(angle);

    rot(0, 0) = cos_ang;
    rot(0, 1) = -sin_ang;
    rot(1, 0) = sin_ang;
    rot(1, 1) = cos_ang;
    return rot;
}

template<class T>
Eigen::Matrix3<T> rotx(T angle) {
    Eigen::Matrix3<T> rotx = Eigen::Matrix3<T>::Zero();
    rotx(0, 0) = 1;

    T cos_ang = std::cos(angle);
    T sin_ang = std::sin(angle);
    rotx(1, 1) = cos_ang;
    rotx(2, 2) = cos_ang;
    rotx(1, 2) = -sin_ang;
    rotx(2, 1) = sin_ang;
    return rotx;
}

template<class T>
Eigen::Matrix3<T> roty(T angle) {
    Eigen::Matrix3<T> roty = Eigen::Matrix3<T>::Zero();
    roty(1, 1) = 1;

    T cos_ang = std::cos(angle);
    T sin_ang = std::sin(angle);
    roty(0, 0) = cos_ang;
    roty(2, 2) = cos_ang;
    roty(0, 2) = sin_ang;
    roty(2, 0) = -sin_ang;
    return roty;
}

template<class T>
Eigen::Matrix3<T> rotz(T angle) {
    Eigen::Matrix3<T> rotz = Eigen::Matrix3<T>::Zero();
    rotz(2, 2) = 1;

    T cos_ang = std::cos(angle);
    T sin_ang = std::sin(angle);
    rotz(0, 0) = cos_ang;
    rotz(1, 1) = cos_ang;
    rotz(0, 1) = -sin_ang;
    rotz(1, 0) = sin_ang;
    return rotz;
}


// Quaternion Operations
template<class T>
Eigen::Vector4<T> quat_unit() {
    Eigen::Vector4<T> quat = Eigen::Vector4<T>::Zero();
    quat[0] = 1;
    return quat;
}


template<class T>
void quat_multiply(const Eigen::Ref<const Eigen::Matrix4X<T>> &quats1,
                   const Eigen::Ref<const Eigen::Matrix4X<T>> &quats2,
                   Eigen::Ref<Eigen::Matrix4X<T>> quats_res) {
    if (quats1.cols() != quats2.cols() or quats_res.cols() != quats1.cols()) {
        throw std::runtime_error("Multiplication must be between the same number of quaternions.");
    }

    quats_res.row(0) = quats1.row(0).cwiseProduct(quats2.row(0)) -
                       quats1.row(1).cwiseProduct(quats2.row(1)) -
                       quats1.row(2).cwiseProduct(quats2.row(2)) -
                       quats1.row(3).cwiseProduct(quats2.row(3));

    quats_res.row(1) = quats1.row(0).cwiseProduct(quats2.row(1)) +
                       quats1.row(1).cwiseProduct(quats2.row(0)) +
                       quats1.row(2).cwiseProduct(quats2.row(3)) -
                       quats1.row(3).cwiseProduct(quats2.row(2));

    quats_res.row(2) = quats1.row(0).cwiseProduct(quats2.row(2)) -
                       quats1.row(1).cwiseProduct(quats2.row(3)) +
                       quats1.row(2).cwiseProduct(quats2.row(0)) +
                       quats1.row(3).cwiseProduct(quats2.row(1));

    quats_res.row(3) = quats1.row(0).cwiseProduct(quats2.row(3)) +
                       quats1.row(1).cwiseProduct(quats2.row(2)) -
                       quats1.row(2).cwiseProduct(quats2.row(1)) +
                       quats1.row(3).cwiseProduct(quats2.row(0));
}

template<class T>
Eigen::Matrix4X<T> quat_multiply(const Eigen::Ref<const Eigen::Matrix4X<T>> &quats1,
                                 const Eigen::Ref<const Eigen::Matrix4X<T>> &quats2) {
    Eigen::Matrix4X<T> quats(4, quats1.cols());
    quat_multiply<T>(quats1, quats2, quats);
    return quats;
}

template<class T>
Eigen::Matrix4X<T> quat_inv(const Eigen::Ref<const Eigen::Matrix4X<T>> &quat) {
    Eigen::Matrix4X<T> quat_inv(quat);

    quat_inv.bottomRows(3) = -quat_inv.bottomRows(3);
    return quat_inv;
}

template<class T>
void quat_inv_inplace(Eigen::Ref<Eigen::Matrix4X<T>>quat) {
    quat.bottomRows(3) = -quat.bottomRows(3);
}

}
}

#endif //ALECTYONMATH_ROTATIONOPERATIONS_HPP
