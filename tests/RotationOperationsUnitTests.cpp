//
// Created by clark on 6/16/17.
//

#include <iostream>
#include <cmath>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include <alectryonmath/AlectryonMath.hpp>
#include <alectryonmath/RotationOperations.hpp>

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

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

