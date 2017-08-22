//
// Created by clark on 4/14/17.
//

#ifndef ALECTRYONMATH_ANGLE_HPP
#define ALECTRYONMATH_ANGLE_HPP

#include <iostream>
#include <cmath>
#include <exception>
#include <stdexcept>
#include "alectryon_math.hpp"

// Function Definitions
namespace Alectryon {
namespace Math {
template<class T>
constexpr T Pi();

template<class T>
constexpr T TwoPi();

template<class T>
constexpr T HalfPi();

/**
 * @brief Constrains an angle to the interval [-pi, pi)
 * @tparam T
 * @param angle angle to constrain
 * @return constrained angle
 */
template<class T>
T constrain_angle_pi(T angle);

/**
 * @brief Constrains an angle to the interval [0, 2 pi)
 * @tparam T
 * @param angle angle to constrain
 * @return constrained angle
 */
template<class T>
T constrain_angle_two_pi(T angle);

/**
 * @brief Calculates the shortest angular distance between two angles
 * @details Angles are treated by their equivalence class (eg. angles in the range [0, 2 pi))
 * So the distance between angle1 and angle2 is the same as the distance between angle1 and angle2 + 2 pi
 * The distance is symmetric. The distance between -0.1 and 0.1 is 0.2. The distance between 2 pi - 0.2 and 2 pi is 0.2
 *
 *
 * @tparam T
 * @param angle1
 * @param angle2
 * @return the shortest angular distance (in a range [0, pi))
 */
template<class T>
T angle_dist(T angle1, T angle2);

/**
 * @brief Interpolates between two angles
 * @details angles are treated as if equal to their equivalence class (eg. angles in the range [0, 2 pi))
 * We obtain the smallest (in terms of magnitude) signed distance between angle1 and angle2
 * The interpolation then returns angle1 + (dist * alpha).
 *
 * @tparam T
 * @param angle1
 * @param angle2
 * @param alpha
 * @return interpolated angle
 */
template<class T>
T angle_interp(T angle1, T angle2, T alpha);

}
}



// Implementation
namespace Alectryon {
namespace Math {


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
    T dist = constrain_angle_two_pi<T>(angle2 - angle1);
    T interp;
    if (dist > Pi<T>()) {
        dist = TwoPi<T>() - dist;
        interp = angle1 - alpha * dist;
    } else {
        interp = angle1 + alpha * dist;
    }

    return interp;
}

}
}

#endif //ALECTRYONMATH_ANGLE_HPP
