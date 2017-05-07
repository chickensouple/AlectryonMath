//
// Created by clark on 4/14/17.
//

#include <iostream>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include <alectryonmath/RotationConversions.hpp>

using namespace Alectryon;

template<class T>
void TestConvertAa4() {
	// checking for no rotation
	Eigen::Matrix<T, 4, 1> aa4(0, 0, 0, 0);

	Eigen::Matrix<T, 3, 3> rot = Transform::aa4_to_rot<T>(aa4);
	Eigen::Matrix<T, 3, 3> rot_ans;
	rot_ans << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	CHECK_FLOATING_ARR(T, 9, rot, rot_ans);
	Eigen::Matrix<T, 4, 1> quat = Transform::aa4_to_quat<T>(aa4);
	Eigen::Matrix<T, 4, 1> quat_ans(1, 0, 0, 0);
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

}

template<class T>
void TestConvertAa3() {
	// checking for no rotation
	Eigen::Matrix<T, 3, 1> aa3(0, 0, 0);

	Eigen::Matrix<T, 4, 1> quat = Transform::aa3_to_quat<T>(aa3);
	Eigen::Matrix<T, 4, 1> quat_ans(1, 0, 0, 0);
	CHECK_FLOATING_ARR(T, 4, quat_ans, quat);
	Eigen::Matrix<T, 4, 1> aa4 = Transform::aa3_to_aa4<T>(aa3);
	Eigen::Matrix<T, 4, 1> aa4_ans(0, 0, 0, 0);
	CHECK_FLOATING_ARR(T, 4, aa4_ans, aa4);
	Eigen::Matrix<T, 3, 3> rot = Transform::aa3_to_rot<T>(aa3);
	Eigen::Matrix<T, 3, 3> rot_ans;
	rot_ans << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	CHECK_FLOATING_ARR(T, 9, rot, rot_ans);
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

