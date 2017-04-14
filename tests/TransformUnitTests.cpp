//
// Created by clark on 4/14/17.
//

#include <iostream>
#include "gtest/gtest.h"
#include "alectryonmath/RotationConversions.h"

TEST(Transformation, Test1) {
    Eigen::Vector3f aa3(1, 0, 0);
    Eigen::Vector4f quat = Transformation::aa3_to_quat(aa3);
    Eigen::Vector4f aa4 = Transformation::aa3_to_aa4(aa3);
    std::cerr << quat;
    EXPECT_EQ(2, 2);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

