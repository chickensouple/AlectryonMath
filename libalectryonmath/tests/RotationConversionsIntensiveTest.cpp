//
// Created by clark on 8/22/17.
//

#include <Eigen/Dense>
#include <iostream>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include <alectryonmath/rotation_conversions.hpp>
#include <fstream>
#include <vector>
#include <functional>
#include <iterator>
#include <algorithm>


using namespace Alectryon;

void RotTest(const Eigen::Ref<const Eigen::MatrixX<double>> &rot,
             const Eigen::Ref<const Eigen::MatrixX<double>> &quat,
             const Eigen::Ref<const Eigen::MatrixX<double>> &aa3,
             const Eigen::Ref<const Eigen::MatrixX<double>> &aa4,
             const Eigen::Ref<const Eigen::MatrixX<double>> &eulerZYX,
             const Eigen::Ref<const Eigen::MatrixX<double>> &eulerXYZ) {

    size_t num_data = rot.cols();
    for (size_t i = 0; i < num_data; i++) {
        Eigen::Vector4<double> vec4;
        Eigen::Matrix<double, 9, 1> rot_vec = rot.col(i);
        Eigen::Map<Eigen::Matrix3<double>> rot_mat(rot_vec.data(), 3, 3);

        // to quat
        Transform::rot_to_quat<double>(rot_mat, vec4);
        Eigen::Vector4<double> quat_ans = quat.col(i);


        std::cout << "Ans: " << quat_ans.transpose() << "\tquat: " << vec4.transpose() << "\n";
        CHECK_FLOATING_ARR(double, 4, quat_ans, vec4);
    }

}

TEST(Transformation, Intensive) {
    std::ifstream input_file;
    input_file.open("rot_data.data", std::ios::binary);


    ASSERT_EQ(input_file.is_open(), true);

    std::vector<char> buffer((std::istreambuf_iterator<char>(input_file)),
                             (std::istreambuf_iterator<char>()));

    size_t num_bytes = buffer.size();
    size_t num_doubles = num_bytes / 8;
    double* data = (double*)buffer.data();
    std::cout << "Read in " << num_bytes << " bytes (" << num_doubles << " doubles)\n";

    size_t data_length = 9 + 4 + 3 + 4 + 3 + 3;
    ASSERT_EQ(num_doubles % data_length, 0);

    size_t num_data = num_doubles / data_length;
    std::cout << "Read in " << num_data << " data points\n";

    double* ptr = data;
    Eigen::Map<Eigen::Matrix<double, 9, Eigen::Dynamic>> rot_data(ptr, 9, num_data);
    ptr += (9 * num_data);
    Eigen::Map<Eigen::Matrix<double, 4, Eigen::Dynamic>> quat_data(ptr, 4, num_data);
    ptr += (4 * num_data);
    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> aa3_data(ptr, 3, num_data);
    ptr += (3 * num_data);
    Eigen::Map<Eigen::Matrix<double, 4, Eigen::Dynamic>> aa4_data(ptr, 4, num_data);
    ptr += (4 * num_data);
    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> eulerZYX_data(ptr, 3, num_data);
    ptr += (3 * num_data);
    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> eulerXYZ_data(ptr, 3, num_data);

    RotTest(rot_data, quat_data, aa3_data, aa4_data, eulerZYX_data, eulerXYZ_data);
}


int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}