//
// This file instantiates templated functions for "float" and "double" types
// Created by clark on 4/14/17.
//
#include "RotationConversions.h"

namespace Transformation {
    Eigen::Matrix<float, 4, 1> aa3_to_quat(const Eigen::Matrix<float, 3, 1>& aa3);
    Eigen::Matrix<double, 4, 1> aa3_to_quat(const Eigen::Matrix<double, 3, 1>& aa3);


    Eigen::Matrix<float, 4, 1> aa3_to_aa4(const Eigen::Matrix<float, 3, 1>& aa3);
    Eigen::Matrix<double, 4, 1> aa3_to_aa4(const Eigen::Matrix<double, 3, 1>& aa3);


    Eigen::Matrix<float, 3, 1> aa4_to_aa3(const Eigen::Matrix<float, 4, 1>& aa4);
    Eigen::Matrix<double, 3, 1> aa4_to_aa3(const Eigen::Matrix<double, 4, 1>& aa4);

}


