//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_ROTATIONCONVERSIONS_HPP
#define ALECTRYONMATH_ROTATIONCONVERSIONS_HPP

#include <Eigen/Dense>
#include <cmath>
#include <type_traits>
#include <alectryonmath/Angle.hpp>

/**
 * Quaternions are represented as length 4 column vectors
 * in the order [w, x, y, z] for a quaternion (w + xi + yj + zk)
 *
 * Angle Axis (aa4) is represented as length 4 column vectors
 * as [\theta, r] where \theta is the angle and r is a unit vector
 *
 * Angle Axis (aa3) is represented as length 3 column vectors
 * Where \theta is the norm of the vector, and the direction is given by the
 * normalization of the vector
 *
 * Euler angles are in either ZYX order or XYZ
 * they are represented as length 3 column vectors
 * where the 1st element is the degree of the angle around the first axis of rotation
 * and so on
 */

// Function Definitions
namespace Alectryon {
namespace Transform {
/**
 * @brief Turns an Angle Axis(3) Representation to Quaternion
 * If Angle Axis has a norm of 0, the unit Quaternion will be returned
 *
 * @tparam T float or double
 * @param aa3 angle axis representation
 * @return Quaternion representation
 */
template<class T>
Eigen::Matrix3<T> aa3_to_rot(const Eigen::Vector3<T> &aa3);

template<class T>
Eigen::Vector4<T> aa3_to_quat(const Eigen::Vector3<T> &aa3);

template<class T>
Eigen::Vector4<T> aa3_to_aa4(const Eigen::Vector3<T> &aa3);

// TODO: aa3_to_eulerZYX, aa3_to_eulerXYZ

template<class T>
Eigen::Matrix3<T> aa4_to_rot(const Eigen::Vector4<T> &aa4);

template<class T>
Eigen::Vector4<T> aa4_to_quat(const Eigen::Vector4<T> &aa4);

template<class T>
Eigen::Matrix3X<T> aa4_to_aa3(const Eigen::Matrix4X<T> &aa4);

// TODO: aa4_to_eulerZYX, aa4_to_eulerXYZ


template<class T>
Eigen::Matrix3X<T> quat_to_rot(const Eigen::Vector4<T> &quat);

template<class T>
Eigen::Vector3<T> quat_to_aa3(const Eigen::Vector4<T> &quat);

template<class T>
Eigen::Vector4<T> quat_to_aa4(const Eigen::Vector4<T> &quat);

// TODO: quat_to_eulerZYX, quat_to_eulerXYZ
template<class T>
Eigen::Matrix3X<T> quat_to_eulerZYX(const Eigen::Matrix4X<T> &quat);

// TODO: rot_to_quat, rot_to_aa3, rot_to_aa4

template<class T>
Eigen::Vector3<T> rot_to_eulerZYX(const Eigen::Matrix3<T> &rot);

// TODO: rot_to_eulerXYZ

// TODO: eulerXYZ_to_rot, eulerXYZ_to_quat, eulerXYZ_to_aa3, eulerXYZ_to_aa4, eulerXYZ_to_ZYX

// TODO: eulerZYX_to_rot, eulerZYX_to_quat, eulerZYX_to_aa3, eulerZYX_to_aa4, eulerZYX_to_XYZ

}
}


// Implementation
namespace Alectryon {
namespace Transform {


template<class T>
Eigen::Matrix3<T> aa3_to_rot(const Eigen::Vector3<T> &aa3) {
    return aa4_to_rot(aa3_to_aa4(aa3));
};

template<class T>
Eigen::Vector4<T> aa3_to_quat(const Eigen::Vector3<T> &aa3) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    T theta = aa3.norm();
    if (Angle::angle_dist<T>(theta, 0) < Math::eps) {
        return Eigen::Vector4<T>(1, 0, 0, 0);
    }

    Eigen::Vector4<T> quat;
    quat[0] = std::cos(0.5 * theta);
    quat.tail(3) = std::sin(0.5 * theta) * aa3 / theta;

    return quat;
};

template<class T>
Eigen::Vector4<T> aa3_to_aa4(const Eigen::Vector3<T> &aa3) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    T theta = aa3.norm();
    if (Angle::angle_dist<T>(theta, 0) < Math::eps) {
        return Eigen::Vector4<T>(0, 0, 0, 0);
    }
    Eigen::Vector4<T> aa4;
    aa4[0] = theta;
    aa4.tail(3) = aa3 / theta;
    return aa4;
};

template<class T>
Eigen::Matrix3<T> aa4_to_rot(const Eigen::Vector4<T> &aa4) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    Eigen::Matrix3<T> rot = Eigen::Matrix3<T>::Identity();
    Eigen::Matrix3<T> K = Math::cross_prod_matrix<T>(aa4.tail(3));
    T theta = aa4[0];
    rot = rot + std::sin(theta) * K + (1 - std::cos(theta)) * (K * K);
    return rot;
};

template<class T>
Eigen::Vector4<T> aa4_to_quat(const Eigen::Vector4<T> &aa4) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    if (aa4.tail(3).squaredNorm() < Math::eps) {
        return Eigen::Vector4<T>(1, 0, 0, 0);
    }
    T half_theta = 0.5 * aa4[0];
    Eigen::Vector4<T> quat;
    quat[0] = std::cos(half_theta);
    quat.tail(3) = std::sin(half_theta) * aa4.tail(3);
    return quat;
};

template<class T>
Eigen::Matrix3X<T> aa4_to_aa3(const Eigen::Matrix4X<T> &aa4) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    Eigen::Matrix3X<T> aa3 = aa4.bottomRows(3);
    Eigen::Matrix<T, 1, Eigen::Dynamic> top = aa4.topRows(1);
    aa3 = aa3.array().rowwise() * top.array();

    return aa3;
}

template<class T>
Eigen::Matrix3<T> quat_to_rot(const Eigen::Vector4<T> &quat) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    Eigen::Matrix3<T> rot;
    T ij = quat[1] * quat[2]; // bc
    T wk = quat[0] * quat[3]; // ad
    T ik = quat[1] * quat[3]; // bd
    T wj = quat[0] * quat[2]; // ac
    T wi = quat[0] * quat[1]; // ab
    T jk = quat[2] * quat[3]; // cd

    T w2 = quat[0] * quat[0];
    T i2 = quat[1] * quat[1];
    T j2 = quat[2] * quat[2];
    T k2 = quat[3] * quat[3];

    rot(0, 1) = 2 * (ij - wk);
    rot(1, 0) = 2 * (ij + wk);
    rot(0, 2) = 2 * (ik + wj);
    rot(2, 0) = 2 * (ik - wj);
    rot(1, 2) = 2 * (jk - wi);
    rot(2, 1) = 2 * (jk + wi);
    rot(0, 0) = w2 + i2 - j2 - k2;
    rot(1, 1) = w2 - i2 + j2 - k2;
    rot(2, 2) = w2 - i2 - j2 + k2;
    return rot;
}

template<class T>
Eigen::Vector3<T> quat_to_aa3(const Eigen::Vector4<T> &quat) {
    return aa4_to_aa3(quat_to_aa4(quat));
};

template<class T>
Eigen::Vector4<T> quat_to_aa4(const Eigen::Vector4<T> &quat) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    Eigen::Vector4<T> aa4;
    // TODO: make sure quat[0] is in range for acos
    aa4[0] = 2 * std::acos(quat[0]);

    T norm = quat.tail(3).norm();
    if (norm < Math::eps) {
        aa4[1] = aa4[2] = aa4[3] = 0;
    } else {
        aa4.tail(3) = quat.tail(3) / norm;
    }
    return aa4;
};

template<class T>
Eigen::Matrix3X<T> quat_to_eulerZYX(const Eigen::Matrix4X<T> &quat) {
    Eigen::Matrix3X<T> eulerZYX(3, quat.cols());

    Eigen::Vector3X<T> qwqx = quat.row(0).cwiseProduct(quat.row(1));
    Eigen::Vector3X<T> qyqz = quat.row(2).cwiseProduct(quat.row(3));
    Eigen::Vector3X<T> qxqx = quat.row(1).square();
    Eigen::Vector3X<T> qyqy = quat.row(2).square();

    Eigen::Vector3X<T> num = 2 * (qwqx + qyqz);
    Eigen::Vector3X<T> den = 1.0 - 2 * (qxqx + qyqy);
    // TODO: finish
}

template<class T>
Eigen::Vector3<T> rot_to_eulerZYX(const Eigen::Matrix3<T> &rot) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    Eigen::Vector3<T> euler;
    T temp = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    if (temp < Math::eps) {
        euler(2) = std::atan2(-rot(1, 2), rot(1, 1)); // roll
        euler(1) = std::atan2(-rot(2, 0), temp); // pitch
        euler(0) = 0; // yaw
    } else {
        euler(2) = std::atan2(rot(2, 1), rot(2, 2)); // roll
        euler(1) = std::atan2(-rot(2, 0), temp); // pitch
        euler(0) = std::atan2(rot(1, 0), rot(0, 0)); // yaw
    }
    return euler;
}

}
}

#endif //ALECTRYONMATH_ROTATIONCONVERSIONS_HPP
