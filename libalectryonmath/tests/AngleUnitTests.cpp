//
// Created by clark on 4/14/17.
//

#include <iostream>
#include <cmath>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include <alectryonmath/alectryon_angle.hpp>

using namespace Alectryon;

template<class T>
void TestConstants() {
    CHECK_FLOATING_POINT(T, Math::Pi<T>(), (T) M_PI);
    CHECK_FLOATING_POINT(T, Math::TwoPi<T>(), (T) (2 * M_PI));
    CHECK_FLOATING_POINT(T, Math::HalfPi<T>(), (T) (0.5 * M_PI));
}

template<class T>
void TestConstrainAngle() {
    T angle1 = 0.0;
    T angle2 = Math::TwoPi<T>() + (T) 0.2;
    T angle3 = Math::Pi<T>() + (T) 0.3;
    T angle4 = Math::TwoPi<T>() * 6 + (T) (-0.2);
    T angle5 = -Math::TwoPi<T>() + (T) 0.1;
    T angle6 = -Math::Pi<T>() + (T) (-0.21);
    T angle7 = -Math::TwoPi<T>() * 4 + (T) 0.1;

    // test when angle doesn't need to be constrained
    CHECK_FLOATING_POINT(T, angle1, Math::constrain_angle_pi<T>(angle1));
    CHECK_FLOATING_POINT(T, angle1, Math::constrain_angle_two_pi<T>(angle1));

    // test when angle is over 2 pi
    CHECK_FLOATING_POINT(T, (T) 0.2, Math::constrain_angle_two_pi(angle2));
    // test when angle is over pi
    CHECK_FLOATING_POINT(T, (T) 0.3 - Math::Pi<T>(), Math::constrain_angle_pi(angle3));

    // test when angle is way over 2 pi
    CHECK_FLOATING_POINT(T, (T) (-0.2) + Math::TwoPi<T>(), Math::constrain_angle_two_pi(angle4));

    // test when angle is below two pi
    CHECK_FLOATING_POINT(T, (T) 0.1, Math::constrain_angle_two_pi(angle5));
    // test when angle is below pi
    CHECK_FLOATING_POINT(T, (T) (-0.21) + Math::Pi<T>(), Math::constrain_angle_pi(angle6));

    // test when angle is way under two pi
    CHECK_FLOATING_POINT(T, (T) 0.1, Math::constrain_angle_pi(angle7));
}

template<class T>
void TestAngleDist() {
    // testing simplest case
    T angle1 = 0.1;
    T angle2 = 0.2;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0.1);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0.1);

    // test when angles are negative
    angle1 = -0.1;
    angle2 = -0.2;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0.1);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0.1);

    // test when one angle is positive, one is negative
    angle1 = -0.1;
    angle2 = 0.2;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0.3);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0.3);

    // test when angles are the same
    angle1 = 1.4;
    angle2 = 1.4;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0);
    angle1 = -2.1;
    angle2 = -2.1;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0);

    // test when angles are around two pi
    angle1 = Math::TwoPi<T>() - 0.05;
    angle2 = Math::TwoPi<T>() + 0.05;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0.1);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0.1);
    angle1 = Math::TwoPi<T>() - 0.05;
    angle2 = 0.05;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0.1);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0.1);

    // test when angles are way above or below two pi
    angle1 = 4 * Math::TwoPi<T>() + 0.5;
    angle2 = 6 * Math::TwoPi<T>() + 0.82;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0.32);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0.32);
    angle1 = 4 * Math::TwoPi<T>() + 0.5;
    angle2 = -6 * Math::TwoPi<T>() + 0.82;
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle1, angle2), (T) 0.32);
    CHECK_FLOATING_POINT(T, Math::angle_dist<T>(angle2, angle1), (T) 0.32);
}

template<class T>
void TestAngleInterp() {
    T angle1 = 0;
    T angle2 = Math::HalfPi<T>();

    // basic test checking that halfway points are equal
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0.5), Math::HalfPi<T>() * 0.5);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0.5), Math::HalfPi<T>() * 0.5);

    // basic tests for other alphas
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0.2), Math::HalfPi<T>() * 0.2);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0.2), Math::HalfPi<T>() * 0.8);

    // tests when angles straddle the 0/2pi crossing
    angle1 = -Math::HalfPi<T>() / 2;
    angle2 = Math::HalfPi<T>() / 2;
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0.5), 0);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0.5), 0);


    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0.1), -0.8 * Math::HalfPi<T>() / 2);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0.1), 0.8 * Math::HalfPi<T>() / 2);

    // test when angles straddle pi
    angle1 = Math::Pi<T>() - (Math::HalfPi<T>() / 2);
    angle2 = Math::Pi<T>() + (Math::HalfPi<T>() / 2);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0.5), Math::Pi<T>());
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0.5), Math::Pi<T>());

    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0.3),
                         Math::Pi<T>() - 0.4 * (Math::HalfPi<T>() / 2));
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0.3),
                         Math::Pi<T>() + 0.4 * (Math::HalfPi<T>() / 2));

    // test corner cases
    angle1 = 0.4;
    angle2 = 0.8;
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0), angle1);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 1), angle2);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0), angle2);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 1), angle1);

    // testing random angles
    angle1 = 0.2;
    angle2 = 0.8;
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 0.1), 0.26);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 0.1), 0.74);

    // testing nonstandard inputs, alpha not in range [0, 1]
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) 2.5), 1.7);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) 2.5), -0.7);

    CHECK_FLOATING_POINT(T, Math::angle_interp(angle1, angle2, (T) -1.5), -0.7);
    CHECK_FLOATING_POINT(T, Math::angle_interp(angle2, angle1, (T) -1.5), 1.7);
}

TEST(Angle, Constants) {
    // testing float version
    TestConstants<float>();

    // testing double version
    TestConstants<double>();

    // testing long double version
    TestConstants<long double>();
}


TEST(Angle, ConstrainAngle) {
    // testing float version
    TestConstrainAngle<float>();

    // testing double version
    TestConstrainAngle<double>();
}

TEST(Angle, AngleDist) {
    // testing float version
    TestAngleDist<float>();

    // testing double version
    TestAngleDist<double>();
}

TEST(Angle, AngleInterp) {
    // testing float version
    TestAngleInterp<float>();

    // testing double version
    TestAngleInterp<double>();
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

