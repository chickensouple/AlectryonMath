//
// Created by clark on 6/16/17.
//

#include <iostream>
#include <cmath>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include <alectryonmath/alectryon_math.hpp>
#include <alectryonmath/rotation_operations.hpp>

using namespace Alectryon::Transform;

TEST(Transform, QuatMultiply) {
    Eigen::Matrix4X<float> quats1(4, 2);
    Eigen::Matrix4X<float> quats2(4, 2);


    quats1 << 1, 1,
            0, 2,
            0, 3,
            0, -1.2;

    quats2 << 2, 1,
            -1, 0,
            0.2, 0,
            1, 0;

    Eigen::Matrix4X<float> quat = quat_multiply<float>(quats1, quats2);
}

template <class T>
void test(Eigen::Ref<Eigen::MatrixX<T>> mat) {
    std::cout << "template\n";
}

void test(Eigen::Ref<Eigen::MatrixXf> mat) {
    std::cout << "float\n";
}

void test(Eigen::Ref<Eigen::MatrixXd> mat) {
    std::cout << "double\n";
}



int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

