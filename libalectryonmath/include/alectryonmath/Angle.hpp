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
T constrain_angle_pi(T angle);

template<class T>
T constrain_angle_two_pi(T angle);

template<class T>
T angle_dist(T angle1, T angle2);

template<class T>
T angle_interp(T angle1, T angle2, T alpha);

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
T constrain_angle_pi(T angle) {
    T constrained_angle = std::fmod(angle + Pi<T>(), TwoPi<T>());
    if (constrained_angle < 0) constrained_angle += TwoPi<T>();
    return constrained_angle - Pi<T>();
}

template<class T>
T constrain_angle_two_pi(T angle) {
    T constrained_angle = std::fmod(angle, TwoPi<T>());
    if (constrained_angle < 0) constrained_angle += TwoPi<T>();
    return constrained_angle;
}

template<class T>
T angle_dist(T angle1, T angle2) {
    T dist = constrain_angle_two_pi<T>(angle1 - angle2);
    if (dist > Pi<T>()) {
        dist = TwoPi<T>() - dist;
    }
    return dist;
}

template<class T>
T angle_interp(T angle1, T angle2, T alpha) {

}

}
}

#endif //ALECTRYONMATH_ANGLE_HPP
