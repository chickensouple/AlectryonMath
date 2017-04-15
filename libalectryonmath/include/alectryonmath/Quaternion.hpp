//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_QUATERNION_HPP
#define ALECTRYONMATH_QUATERNION_HPP

#include <Eigen/Dense>
#include <alectryonmath/AlectryonMath.hpp>

// Function Definitions
namespace Alectryon {
namespace Transform {

template <class T>
Eigen::Vector4<T> quat_multiply(const Eigen::Vector4<T> &quat1, const Eigen::Vector4<T> &quat2);

template <class T>
Eigen::Vector4<T> quat_inv(const Eigen::Vector4<T> &quat);

template <class T>
void quat_inv_inplace(Eigen::Vector4<T> &quat);

}
}

// Implementation
namespace Alectryon {
namespace Transform {

template <class T>
Eigen::Vector4<T> quat_multiply(const Eigen::Vector4<T> &quat1, const Eigen::Vector4<T> &quat2) {
	Eigen::Vector4<T> quat;
	quat(0) = (quat1(0)*quat2(0)) - (quat1(1)*quat2(1)) - (quat1(2)*quat2(2)) - (quat1(3)*quat2(3));
	quat(1) = (quat1(0)*quat2(1)) + (quat1(1)*quat2(0)) + (quat1(2)*quat2(3)) - (quat1(3)*quat2(2));
	quat(2) = (quat1(0)*quat2(2)) - (quat1(1)*quat2(3)) + (quat1(2)*quat2(0)) + (quat1(3)*quat2(1));
	quat(3) = (quat1(0)*quat2(3)) + (quat1(1)*quat2(2)) - (quat1(2)*quat2(1)) + (quat1(3)*quat2(0));

	return quat;
}

template <class T>
Eigen::Vector4<T> quat_inv(const Eigen::Vector4<T> &quat) {
	Eigen::Vector4<T> quat_inv = quat;
	quat_inv.tail(3) = -quat_inv.tail(3);
	return quat_inv;
}

template <class T>
void quat_inv_inplace(Eigen::Vector4<T> &quat) {
	quat.tail(3) = -quat.tail(3);
}

}
}

#endif //ALECTRYONMATH_QUATERNION_HPP
