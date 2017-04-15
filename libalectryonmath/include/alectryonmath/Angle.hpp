//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_ANGLE_HPP
#define ALECTRYONMATH_ANGLE_HPP

#include <iostream>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <alectryonmath/AlectryonMath.hpp>


// Function Definitions
namespace Alectryon {
namespace Angle {
template<class T>
constexpr T Pi();

template<class T>
constexpr T TwoPi();

template<class T>
constexpr T HalfPi();

template<class T>
T constrain_angle(T angle, T min, T max);

template<class T>
T constrain_angle_pi(T angle);

template<class T>
T constrain_angle_two_pi(T angle);

template<class T>
T angle_dist(T angle1, T angle2);
}
}


// Implementation
namespace Alectryon {
namespace Angle {

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

template<class T>
T constrain_angle(T angle, T min, T max) {
	if ((min > max) or (std::fabs(max - min - TwoPi<T>()) > Math::eps)) {
		throw std::invalid_argument("max must be 2*pi bigger than min");
	}

	while (angle < min) {
		angle += TwoPi<T>();
	}
	while (angle > max) {
		angle -= TwoPi<T>();
	}
	return angle;
}

template<class T>
T constrain_angle_pi(T angle) {
	return constrain_angle<T>(angle, -Pi<T>(), Pi<T>());
}

template<class T>
T constrain_angle_two_pi(T angle) {
	return constrain_angle<T>(angle, (T) 0, TwoPi<T>());
}

template<class T>
T angle_dist(T angle1, T angle2) {
	T dist = constrain_angle_two_pi(angle1 - angle2);
	if (dist > Pi<T>()) {
		dist = TwoPi<T>() - dist;
	}
	return dist;
}

}
}

#endif //ALECTRYONMATH_ANGLE_HPP
