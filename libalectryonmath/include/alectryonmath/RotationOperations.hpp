//
// Created by clark on 4/15/17.
//

#ifndef ALECTYONMATH_ROTATIONOPERATIONS_HPP
#define ALECTYONMATH_ROTATIONOPERATIONS_HPP

#include <cmath>
#include <alectryonmath/AlectryonMath.hpp>

namespace Alectryon {
namespace Transform {

// Function Definitions
template <class T>
Eigen::Matrix3<T> rotx(T angle);

template <class T>
Eigen::Matrix3<T> roty(T angle);

template <class T>
Eigen::Matrix3<T> rotz(T angle);

}
}


// Implementation
namespace Alectryon {
namespace Transform {

template <class T>
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

template <class T>
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

template <class T>
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


}
}

#endif //ALECTYONMATH_ROTATIONOPERATIONS_HPP
