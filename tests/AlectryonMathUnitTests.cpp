//
// Created by clark on 4/14/17.
//

#include <iostream>
#include <cmath>
#include <functional>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include <alectryonmath/AlectryonMath.hpp>

using namespace Alectryon;

template<class T>
void NormalizeTest() {
    Eigen::MatrixX<T> mat(3, 2);
    mat << 1,   -0.5,
           2,   1,
           1.2, -0.2;

    Eigen::MatrixX<T> mat_ans(3, 2);
    mat_ans << 0.39405520311955033463, -0.44022545316281191941,
               0.78811040623910066927,  0.88045090632562383881,
               0.47286624374346042377, -0.17609018126512479552;

    Eigen::MatrixX<T> mat_norm(3, 2);
    Math::normalize_mat<T>(mat, mat_norm);
    CHECK_FLOATING_ARR(T, 6, mat_ans, mat_norm);

    mat_norm = mat;
    mat_norm = Math::normalize_mat<T>(mat);
    CHECK_FLOATING_ARR(T, 6, mat_ans, mat_norm);

    mat_norm = mat;
    Math::normalize_mat_inplace<T>(mat_norm);
    CHECK_FLOATING_ARR(T, 6, mat_ans, mat_norm);
}

TEST(Math, Normalize) {
    NormalizeTest<float>();
    NormalizeTest<double>();
}

template<class T>
void Atan2Test() {
    // testing 49 angles evenly spaced around unit circle
    Eigen::VectorX<T> angles_vec = Eigen::ArrayX<T>::LinSpaced(49, 0, Math::TwoPi<T>());
    Eigen::Map<Eigen::Matrix<T, 7, 7>> angles(angles_vec.data(), 7, 7);

    Eigen::Matrix<T, 7, 7> sin_angles = angles.array().sin();
    Eigen::Matrix<T, 7, 7> cos_angles = angles.array().cos();

    Eigen::Matrix<T, 7, 7> atan2_angles_ans;
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            atan2_angles_ans(i, j) = std::atan2(sin_angles(i, j), cos_angles(i, j));
    Eigen::Matrix<T, 7, 7> atan2_angles = Math::atan2<T>(sin_angles, cos_angles);

    CHECK_FLOATING_ARR(T, 49, atan2_angles_ans, atan2_angles);

    // testing boundary cases
    T inf = std::numeric_limits<T>::infinity();
    int num = 16;
    Eigen::VectorX<T> y(num);
    Eigen::VectorX<T> x(num);
    y << -inf, inf, 1.2, -1, inf, inf, inf,  0,   0.2, -4.6,
            -inf, -inf, -inf, 0,    0.2,  -4.6;
    x << -inf, inf, 0,   0,  0,   0.2, -6.1, inf, inf, inf,
            0,    0.2,  -6.1, -inf, -inf, -inf;

    Eigen::VectorX<T> atan2_test = Math::atan2<T>(y, x);
    Eigen::VectorX<T> atan2_ans(num);
    for (int i = 0; i < num; i++) atan2_ans[i] = std::atan2(y[i], x[i]);
    CHECK_FLOATING_ARR(T, num, atan2_ans, atan2_test);
}

TEST(Math, Atan2) {
    Atan2Test<float>();
    Atan2Test<double>();
}


int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

