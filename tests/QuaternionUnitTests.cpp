//
// Created by clark on 6/16/17.
//

#include <iostream>
#include <cmath>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include "alectryonmath/Quaternion.hpp"

using namespace Alectryon::Transform;

TEST(quaternion, multiplication) {
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

    Eigen::Matrix4X<float> quat = quat_multiply(quats1, quats2);


//    std::cout << quat << "\n";

    Eigen::Matrix4X<float> test;
    quat_normalize(test, quats1);
    std::cout << test << "\n";

//    quat_normalize_inplace(quats1);
    std::cout << quats1 << "\n";

}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

