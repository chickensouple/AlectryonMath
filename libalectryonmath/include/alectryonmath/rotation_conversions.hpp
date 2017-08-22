//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_ROTATIONCONVERSIONS_HPP
#define ALECTRYONMATH_ROTATIONCONVERSIONS_HPP

#include <Eigen/Dense>
#include <cmath>
#include <type_traits>
#include <alectryonmath/alectryon_math.hpp>

/**
 * Rotation Matrices are repented by 3 by 3 matrices
 *
 * Quaternions are represented as length 4 column vectors
 * in the order [w, x, y, z] for a quaternion (w + xi + yj + zk)
 *
 * Axis Angle (aa3) is represented as length 3 column vectors
 * Where \theta is the norm of the vector, and the direction is given by the
 * normalization of the vector

 * Axis Angle (aa4) is represented as length 4 column vectors
 * as [\theta, r] where \theta is the angle and r is a unit vector
 *
 * Euler angles are in either ZYX order or XYZ
 * they are represented as length 3 column vectors
 * where the 1st element is the degree of the angle around the first axis of rotation
 * and so on
 */

// Function Definitions
namespace Alectryon {
namespace Transform {

// Rotation Matrix Conversions
// TODO: rot_to_quat, rot_to_aa3, rot_to_aa4
// TODO: rot_to_eulerXYZ

template<class T>
void rot_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix3<T>> &rot, Eigen::Ref<Eigen::Vector3<T>> eulerZYX);
 
template<class T>
Eigen::Vector3<T> rot_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix3<T>> &rot);

// Quaternion Conversions
template<class T>
void quat_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &quat, Eigen::Ref<Eigen::Matrix3<T>> rot);

template<class T>
Eigen::Matrix3<T> quat_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &quat);

template<class T>
Eigen::Matrix3<T> quat_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &quat);

template<class T>
void quat_to_aa3(const Eigen::Ref<const Eigen::Vector4<T>> &quat, Eigen::Ref<Eigen::Vector3<T>> aa3);

template<class T>
void quat_to_aa4(const Eigen::Ref<const Eigen::Vector4<T>> &quat, Eigen::Ref<Eigen::Vector4<T>> aa4);

template<class T>
Eigen::Vector4<T> quat_to_aa4(const Eigen::Ref<const Eigen::Vector4<T>> &quat);

template<class T>
void quat_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix4X<T>> &quat, Eigen::Ref<Eigen::Matrix3X<T>> eulerZYX);

template<class T>
Eigen::Matrix3X<T> quat_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix4X<T>> &quat);

// TODO: quat_to_eulerXYZ


// Axis Angle(3) Conversions
template<class T>
void aa3_to_rot(const Eigen::Ref<const Eigen::Vector3<T>> &aa3, Eigen::Ref<Eigen::Matrix3<T>> rot);

/**
 * @brief Turns an Axis Angle(3) Representation to Quaternion
 * If Axis Angle has a norm of 0, the unit Quaternion will be returned
 *
 * @tparam T float or double
 * @param aa3 angle axis representation
 * @return Quaternion representation
 */
template<class T>
Eigen::Matrix3<T> aa3_to_rot(const Eigen::Ref<const Eigen::Vector3<T>> &aa3);

template<class T>
void aa3_to_quat(const Eigen::Ref<const Eigen::Vector3<T>> &aa3, Eigen::Ref<Eigen::Vector4<T>> quat);

template<class T>
Eigen::Vector4<T> aa3_to_quat(const Eigen::Ref<const Eigen::Vector3<T>> &aa3);

template<class T>
void aa3_to_aa4(const Eigen::Ref<const Eigen::Vector3<T>> &aa3, Eigen::Ref<Eigen::Vector4<T>> aa4);

template<class T>
Eigen::Vector4<T> aa3_to_aa4(const Eigen::Ref<const Eigen::Vector3<T>> &aa3);

// TODO: aa3_to_eulerZYX, aa3_to_eulerXYZ


// Axis Angle(4) Conversions
template<class T>
void aa4_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &aa4, Eigen::Ref<Eigen::Matrix3<T>> rot);

template<class T>
Eigen::Matrix3<T> aa4_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &aa4);

template<class T>
void aa4_to_quat(const Eigen::Ref<const Eigen::Vector4<T>>& aa4, Eigen::Ref<Eigen::Vector4<T>> quat);

template<class T>
Eigen::Vector4<T> aa4_to_quat(const Eigen::Ref<const Eigen::Vector4<T>> &aa4);

template <class T>
void aa4_to_aa3(const Eigen::Ref<const Eigen::Matrix4X<T>> &aa4, Eigen::Ref<Eigen::Matrix3X<T>> aa3);

template<class T>
Eigen::Matrix3X<T> aa4_to_aa3(const Eigen::Ref<const Eigen::Matrix4X<T>> &aa4);

// TODO: aa4_to_eulerZYX, aa4_to_eulerXYZ

// EulerZYX Conversions
// TODO: eulerZYX_to_rot, eulerZYX_to_quat, eulerZYX_to_aa3, eulerZYX_to_aa4, eulerZYX_to_XYZ

// EulerXYZ Conversions
// TODO: eulerXYZ_to_rot, eulerXYZ_to_quat, eulerXYZ_to_aa3, eulerXYZ_to_aa4, eulerXYZ_to_ZYX
}
}


// Implementation
namespace Alectryon {
namespace Transform {
// Rotation Matrix Conversions
template<class T>
void rot_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix3<T>> &rot, Eigen::Ref<Eigen::Vector3<T>> eulerZYX) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    T temp = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    if (temp < Math::eps) {
        eulerZYX(2) = std::atan2(-rot(1, 2), rot(1, 1)); // roll
        eulerZYX(1) = std::atan2(-rot(2, 0), temp); // pitch
        eulerZYX(0) = 0; // yaw
    } else {
        eulerZYX(2) = std::atan2(rot(2, 1), rot(2, 2)); // roll
        eulerZYX(1) = std::atan2(-rot(2, 0), temp); // pitch
        eulerZYX(0) = std::atan2(rot(1, 0), rot(0, 0)); // yaw
    }    
}

template<class T>
Eigen::Vector3<T> rot_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix3<T>> &rot) {
    Eigen::Vector3<T> eulerZYX;
    rot_to_eulerZYX<T>(rot, eulerZYX);
    return eulerZYX;
}

// Quaternion Conversions
template<class T>
void quat_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &quat, Eigen::Ref<Eigen::Matrix3<T>> rot) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

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
}

template<class T>
Eigen::Matrix3<T> quat_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &quat) {
    Eigen::Matrix3<T> rot;
    quat_to_rot<T>(quat, rot);
    return rot;
}

template<class T>
void quat_to_aa3(const Eigen::Ref<const Eigen::Vector4<T>> &quat, Eigen::Ref<Eigen::Vector3<T>> aa3) {
    aa4_to_aa3<T>(quat_to_aa4<T>(quat), aa3);
}

template<class T>
Eigen::Vector3<T> quat_to_aa3(const Eigen::Ref<const Eigen::Vector4<T>> &quat) {
    Eigen::Vector3<T> aa3;
    quat_to_aa3<T>(quat, aa3);
    return aa3;
};

template<class T>
void quat_to_aa4(const Eigen::Ref<const Eigen::Vector4<T>> &quat, Eigen::Ref<Eigen::Vector4<T>> aa4) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    // TODO: make sure quat[0] is in range for acos
    aa4[0] = 2 * std::acos(quat[0]);

    T norm = quat.tail(3).norm();
    if (norm < Math::eps) {
        aa4[1] = aa4[2] = aa4[3] = 0;
    } else {
        aa4.tail(3) = quat.tail(3) / norm;
    }
}

template<class T>
Eigen::Vector4<T> quat_to_aa4(const Eigen::Ref<const Eigen::Vector4<T>> &quat) {
    Eigen::Vector4<T> aa4;
    quat_to_aa4<T>(quat, aa4);
    return aa4;
};

template<class T>
void quat_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix4X<T>> &quat, Eigen::Ref<Eigen::Matrix3X<T>> eulerZYX) {
    Eigen::VectorX<T> qwqx = quat.row(0).cwiseProduct(quat.row(1));
    Eigen::VectorX<T> qyqz = quat.row(2).cwiseProduct(quat.row(3));
    Eigen::VectorX<T> qxqx = quat.row(1).square();
    Eigen::VectorX<T> qyqy = quat.row(2).square();

    Eigen::VectorX<T> num = 2 * (qwqx + qyqz);
    Eigen::VectorX<T> den = 1.0 - 2 * (qxqx + qyqy);
    eulerZYX.row(2) = Math::atan2(num, den);


    Eigen::VectorX<T> qwqy = quat.row(0).cwiseProduct(quat.row(2));
    Eigen::VectorX<T> qzqx = quat.row(3).cwiseProduct(quat.row(1));
    eulerZYX.row(1) = Eigen::asin(2 * (qwqy - qzqx).array());

    Eigen::VectorX<T> qwqz = quat.row(0).cwiseProduct(quat.row(3));
    Eigen::VectorX<T> qxqy = quat.row(1).cwiseProduct(quat.row(2));
    Eigen::VectorX<T> qzqz = quat.row(3).square();

    num = 2 * (qwqz + qxqy);
    den = 1.0 - 2 * (qyqy + qzqz);
    eulerZYX.row(0) = Math::atan2(num, den);    
}

template<class T>
Eigen::Matrix3X<T> quat_to_eulerZYX(const Eigen::Ref<const Eigen::Matrix4X<T>> &quat) {
    Eigen::Matrix3X<T> eulerZYX(3, quat.cols());
    quat_to_eulerZYX<T>(quat, eulerZYX);
    return eulerZYX;
}

// Axis Angle(3) Conversions
template<class T>
void aa3_to_rot(const Eigen::Ref<const Eigen::Vector3<T>> &aa3, Eigen::Ref<Eigen::Matrix3<T>> rot) {
   aa4_to_rot<T>(aa3_to_aa4<T>(aa3), rot);
}

template<class T>
Eigen::Matrix3<T> aa3_to_rot(const Eigen::Ref<const Eigen::Vector3<T>> &aa3) {
    Eigen::Matrix3<T> rot;
    aa3_to_rot<T>(aa3, rot);
    return rot;
};

template<class T>
void aa3_to_quat(const Eigen::Ref<const Eigen::Vector3<T>> &aa3, Eigen::Ref<Eigen::Vector4<T>> quat) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    T theta = aa3.norm();
    if (Math::angle_dist<T>(theta, 0) < Math::eps) {
        quat = Eigen::Vector4<T>(1, 0, 0, 0);
        return;
    }

    quat[0] = std::cos(0.5 * theta);
    quat.tail(3) = std::sin(0.5 * theta) * aa3 / theta;
}

template<class T>
Eigen::Vector4<T> aa3_to_quat(const Eigen::Ref<const Eigen::Vector3<T>> &aa3) {
    Eigen::Vector4<T> quat;
    aa3_to_quat<T>(aa3, quat);
    return quat;
};

template<class T>
void aa3_to_aa4(const Eigen::Ref<const Eigen::Vector3<T>> &aa3, Eigen::Ref<Eigen::Vector4<T>> aa4) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    T theta = aa3.norm();
    if (Math::angle_dist<T>(theta, 0) < Math::eps) {
        aa4 = Eigen::Vector4<T>(0, 0, 0, 0);
        return;
    }
    aa4[0] = theta;
    aa4.tail(3) = aa3 / theta;
}

template<class T>
Eigen::Vector4<T> aa3_to_aa4(const Eigen::Ref<const Eigen::Vector3<T>> &aa3) {
    Eigen::Vector4<T> aa4;
    aa3_to_aa4<T>(aa3, aa4);
    return aa4;
};

// Axis Angle(4) Conversions
template<class T>
void aa4_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &aa4, Eigen::Ref<Eigen::Matrix3<T>> rot) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    rot = Eigen::Matrix3<T>::Identity();
    Eigen::Matrix3<T> K = Math::cross_prod_mat<T>(aa4.tail(3));
    T theta = aa4[0];
    rot = rot + std::sin(theta) * K + (1 - std::cos(theta)) * (K * K);
}

template<class T>
Eigen::Matrix3<T> aa4_to_rot(const Eigen::Ref<const Eigen::Vector4<T>> &aa4) {
    Eigen::Matrix3<T> rot;
    aa4_to_rot<T>(aa4, rot);
    return rot;
};

template<class T>
void aa4_to_quat(const Eigen::Ref<const Eigen::Vector4<T>>& aa4, Eigen::Ref<Eigen::Vector4<T>> quat) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    if (aa4.tail(3).squaredNorm() < Math::eps) {
        quat = Eigen::Vector4<T>(1, 0, 0, 0);
        return;
    }
    T half_theta = 0.5 * aa4[0];
    quat[0] = std::cos(half_theta);
    quat.tail(3) = std::sin(half_theta) * aa4.tail(3);
}

template<class T>
Eigen::Vector4<T> aa4_to_quat(const Eigen::Ref<const Eigen::Vector4<T>> &aa4) {
    Eigen::Vector4<T> quat;
    aa4_to_quat<T>(aa4, quat);
    return quat;
};

template <class T>
void aa4_to_aa3(const Eigen::Ref<const Eigen::Matrix4X<T>> &aa4, Eigen::Ref<Eigen::Matrix3X<T>> aa3) {
    static_assert(std::is_floating_point<T>::value, "Type must be floating point");

    aa3 = aa4.bottomRows(3);
    Eigen::RowVectorX<T> top = aa4.topRows(1);
    aa3 = aa3.array().rowwise() * top.array();
}

template<class T>
Eigen::Matrix3X<T> aa4_to_aa3(const Eigen::Ref<const Eigen::Matrix4X<T>> &aa4) {
    Eigen::Matrix3X<T> aa3(3, aa4.cols());
    aa4_to_aa3<T>(aa4, aa3);
    return aa3;
}


}
}

#endif //ALECTRYONMATH_ROTATIONCONVERSIONS_HPP
