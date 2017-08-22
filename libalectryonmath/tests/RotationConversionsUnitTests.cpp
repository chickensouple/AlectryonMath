//
// Created by clark on 4/14/17.
//

#include <iostream>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include <alectryonmath/rotation_conversions.hpp>

using namespace Alectryon;

template<class T>
void TestConvertQuat() {

}

template<class T>
void TestConvertAa3() {
    // checking for no rotation
    Eigen::Matrix<T, 3, 1> aa3(0, 0, 0);

    Eigen::Vector4<T> quat = Transform::aa3_to_quat<T>(aa3);
    Eigen::Vector4<T> quat_ans(1, 0, 0, 0);

    CHECK_FLOATING_ARR(T, 4, quat_ans, quat);
    Eigen::Vector4<T> aa4 = Transform::aa3_to_aa4<T>(aa3);
    Eigen::Vector4<T> aa4_ans(0, 0, 0, 0);
    CHECK_FLOATING_ARR(T, 4, aa4_ans, aa4);
    Eigen::Matrix3<T> rot = Transform::aa3_to_rot<T>(aa3);
    Eigen::Matrix3<T> rot_ans;
    rot_ans << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    CHECK_FLOATING_ARR(T, 9, rot, rot_ans);

    // test
    rot = Eigen::Matrix3<T>::Zero();
//    Transform::aa3_to_rot<T>(rot, aa3);
}

template<class T>
void TestConvertAa4() {
    // checking for no rotation
    Eigen::Vector4<T> aa4(0, 0, 0, 0);

    Eigen::Matrix3<T> rot = Transform::aa4_to_rot<T>(aa4);
    Eigen::Matrix3<T> rot_ans;
    rot_ans << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    CHECK_FLOATING_ARR(T, 9, rot, rot_ans);
    Eigen::Vector4<T> quat = Transform::aa4_to_quat<T>(aa4);
    Eigen::Vector4<T> quat_ans(1, 0, 0, 0);
    CHECK_FLOATING_ARR(T, 4, quat_ans, quat);
    Eigen::Matrix<T, 3, 1> aa3 = Transform::aa4_to_aa3<T>(aa4);
    Eigen::Matrix<T, 3, 1> aa3_ans(0, 0, 0);
    CHECK_FLOATING_ARR(T, 3, aa3_ans, aa3);


    // checking for no rotation but nonzero theta
    aa4 << 4.2, 0, 0, 0;

    rot = Transform::aa4_to_rot<T>(aa4);
    rot_ans << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    CHECK_FLOATING_ARR(T, 9, rot, rot_ans);
    quat = Transform::aa4_to_quat<T>(aa4);
    CHECK_FLOATING_ARR(T, 4, quat_ans, quat);
    aa3 = Transform::aa4_to_aa3<T>(aa4);
    CHECK_FLOATING_ARR(T, 3, aa3_ans, aa3);

    // random axis angle
    aa4 << 1.124, 0.44715956557060893, 0.6324746330560239, 0.6324746330560239;

    quat_ans << 0.8461910448161652, 0.23828223039917737, 0.33703285770746444, 0.33703285770746444;
    quat = Transform::aa4_to_quat<T>(aa4);
    CHECK_FLOATING_ARR(T, 4, quat_ans, quat);
    rot_ans << 0.5456354113021601, -0.40977048989702813, 0.731006254106401,
            0.731006254106401, 0.6592608630030667, -0.17608228465629228,
            -0.40977048989702813, 0.6304468733541322, 0.6592608630030667;
    rot = Transform::aa4_to_rot<T>(aa4);
    CHECK_FLOATING_ARR(T, 9, rot, rot_ans);
    aa3_ans << 0.50260735170136443732, 0.7109014875549708636, 0.7109014875549708636;
    aa3 = Transform::aa4_to_aa3<T>(aa4);
    CHECK_FLOATING_ARR(T, 3, aa3_ans, aa3);


    // transforming multiple
    Eigen::Matrix<T, 4, 2> aa4_mult;
    aa4_mult << 0, 2.4,
            0, 1,
            0, 0,
            0, 0;
    Eigen::Matrix<T, 3, 2> aa3_mult = Transform::aa4_to_aa3<T>(aa4_mult);
    Eigen::Matrix<T, 3, 2> aa3_mult_ans;
    aa3_mult_ans << 0, 2.4,
            0, 0,
            0, 0;
    CHECK_FLOATING_ARR(T, 6, aa3_mult_ans, aa3_mult);
}

TEST(Transformation, Convert) {
    TestConvertAa4<float>();
    TestConvertAa4<double>();

    TestConvertAa3<float>();
    TestConvertAa3<double>();
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

