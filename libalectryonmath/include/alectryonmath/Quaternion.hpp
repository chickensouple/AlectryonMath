//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_QUATERNION_HPP
#define ALECTRYONMATH_QUATERNION_HPP

#include <Eigen/Dense>
#include <alectryonmath/AlectryonMath.hpp>

// Quaternions will be represented as vectors of 4 elements (w, x, y, z)
namespace Alectryon {
namespace Transform {

template<class T>
void quat_multiply(Eigen::Matrix4X<T> &quats_res, const Eigen::Matrix4X<T> &quats1, const Eigen::Matrix4X<T> &quats2);

template<class T>
Eigen::Matrix4X<T> quat_multiply(const Eigen::Matrix4X<T> &quats1, const Eigen::Matrix4X<T> &quats2);

template<class T>
void quat_normalize(Eigen::Matrix4X<T> &dest, const Eigen::Matrix4X<T> &src);

template<class T>
Eigen::Matrix4X<T> quat_normalize(const Eigen::Matrix4X<T> &src);

template<class T>
void quat_normalize_inplace(Eigen::Matrix4X<T> &quat);

template<class T>
Eigen::Matrix4X<T> quat_inv(const Eigen::Matrix4X<T> &quat);

template<class T>
void quat_inv_inplace(Eigen::Matrix4X<T> &quat);

}
}

// Implementation
namespace Alectryon {
namespace Transform {

template<class T>
void quat_multiply(Eigen::Matrix4X<T> &quats_res, const Eigen::Matrix4X<T> &quats1, const Eigen::Matrix4X<T> &quats2) {
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
Eigen::Matrix4X<T> quat_multiply(const Eigen::Matrix4X<T> &quats1, const Eigen::Matrix4X<T> &quats2) {

    Eigen::Matrix4X<T> quats(4, quats1.cols());
    quat_multiply(quats, quats1, quats2);
    return quats;
}

template<class T>
void quat_normalize(Eigen::Matrix4X<T> &dest, const Eigen::Matrix4X<T> &src) {
    Eigen::VectorX<T> norms = src.colwise().norm();

    dest = src.array().rowwise() / norms.transpose().array();
}

template<class T>
Eigen::Matrix4X<T> quat_normalize(const Eigen::Matrix4X<T> &src) {
    Eigen::Matrix4X<T> dest;
    quat_normalize(dest, src);
    return dest;
}

template<class T>
void quat_normalize_inplace(Eigen::Matrix4X<T> &quat) {
    quat_normalize(quat, quat);
}

template<class T>
Eigen::Matrix4X<T> quat_inv(const Eigen::Matrix4X<T> &quat) {
    Eigen::Matrix4X<T> quat_inv(quat);

    quat_inv.bottomRows(3) = -quat_inv.bottomRows(3);
    return quat_inv;
}

template<class T>
void quat_inv_inplace(Eigen::Matrix4X<T> &quat) {
    quat.bottomRows(3) = -quat.bottomRows(3);
}

}
}

#endif //ALECTRYONMATH_QUATERNION_HPP
