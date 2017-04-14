//
// Created by clark on 4/14/17.
//

#ifndef ROTATIONS_ROTATIONCONVERSIONS_H
#define ROTATIONS_ROTATIONCONVERSIONS_H

#include <Eigen/Dense>
#include <cmath>
#include <type_traits>

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
 */

// Function Definitions
namespace Transformation {

    /**
     * @brief Turns an Angle Axis(3) Representation to Quaternion
     * If Angle Axis has a norm of 0, the unit Quaternion will be returned
     *
     * @tparam T float or double
     * @param aa3 angle axis representation
     * @return Quaternion representation
     */
    template <class T>
    Eigen::Matrix<T, 4, 1> aa3_to_quat(const Eigen::Matrix<T, 3, 1> &aa3);

    template <class T>
    Eigen::Matrix<T, 4, 1> aa3_to_aa4(const Eigen::Matrix<T, 3, 1>& aa3);

    template <class T>
    Eigen::Matrix<T, 3, 1> aa4_to_aa3(const Eigen::Matrix<T, 4, 1>& aa4);

    template <class T>
    Eigen::Matrix<T, 4, 1> aa4_to_quat(const Eigen::Matrix<T, 4, 1>& aa4);

    template <class T>
    Eigen::Matrix<T, 3, 1> quat_to_aa3(const Eigen::Matrix<T, 4, 1>& quat);

}


// Implementation
namespace Transformation {
    template<class T>
    Eigen::Matrix<T, 4, 1> aa3_to_quat(const Eigen::Matrix<T, 3, 1> &aa3) {
        static_assert(std::is_floating_point<T>::value, "Type must be floating point");

        T theta = aa3.norm();
        if (theta == 0) {
            return Eigen::Matrix<T, 4, 1>(1, 0, 0, 0);
        }

        Eigen::Matrix<T, 4, 1> quat;
        quat[0] = std::cos(0.5 * theta);
        quat.tail(3) = std::sin(0.5 * theta) * aa3 / theta;

        return quat;
    };

    template <class T>
    Eigen::Matrix<T, 4, 1> aa3_to_aa4(const Eigen::Matrix<T, 3, 1>& aa3) {
        static_assert(std::is_floating_point<T>::value, "Type must be floating point");

        T theta = aa3.norm();
        if (theta == 0) {
            return Eigen::Matrix<T, 4, 1>(0, 0, 0, 0);
        }
        Eigen::Matrix<T, 4, 1> aa4;
        aa4[0] = theta;
        aa4.tail(3) = aa3 / theta;
        return aa4;
    };

    template <class T>
    Eigen::Matrix<T, 3, 1> aa4_to_aa3(const Eigen::Matrix<T, 4, 1>& aa4) {
        static_assert(std::is_floating_point<T>::value, "Type must be floating point");

        Eigen::Matrix<T, 3, 1> aa3 = aa4[0] * aa4.tail(3);
        return aa3;
    };

    template <class T>
    Eigen::Matrix<T, 4, 1> aa4_to_quat(const Eigen::Matrix<T, 4, 1>& aa4) {
        static_assert(std::is_floating_point<T>::value, "Type must be floating point");

        Eigen::Matrix<T, 4, 1> quat;
        quat[0] = std::cos(0.5 * aa4[0]);
        quat.tail(3) = aa4.tail(3);
    };

    template <class T>
    Eigen::Matrix<T, 3, 1> quat_to_aa3(const Eigen::Matrix<T, 4, 1>& quat) {
        static_assert(std::is_floating_point<T>::value, "Type must be floating point");


    };
}

#endif //ROTATIONS_ROTATIONCONVERSIONS_H
