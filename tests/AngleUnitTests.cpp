//
// Created by clark on 4/14/17.
//

#include <iostream>
#include <cmath>
#include "gtest/gtest.h"
#include "TestHelpers.h"
#include "alectryonmath/Angle.hpp"

using namespace Alectryon;

template<class T>
void TestConstants() {
	CHECK_FLOATING_POINT(T, Angle::Pi<T>(), (T) M_PI);
	CHECK_FLOATING_POINT(T, Angle::TwoPi<T>(), (T) (2 * M_PI));
	CHECK_FLOATING_POINT(T, Angle::HalfPi<T>(), (T) (0.5 * M_PI));
}

template<class T>
void TestConstrainAngle() {
	T angle1 = 0.0;
	T angle2 = Angle::TwoPi<T>() + (T) 0.2;
	T angle3 = Angle::Pi<T>() + (T) 0.3;
	T angle4 = Angle::TwoPi<T>() * 6 + (T) (-0.2);
	T angle5 = -Angle::TwoPi<T>() + (T) 0.1;
	T angle6 = -Angle::Pi<T>() + (T) (-0.21);
	T angle7 = -Angle::TwoPi<T>() * 4 + (T) 0.1;

	// test when angle doesn't need to be constrained
	CHECK_FLOATING_POINT(T, angle1, Angle::constrain_angle_pi<T>(angle1));
	CHECK_FLOATING_POINT(T, angle1, Angle::constrain_angle_two_pi<T>(angle1));

	// test when angle is constrained with custom
	CHECK_FLOATING_POINT(T, angle1 + Angle::TwoPi<T>(),
	                     Angle::constrain_angle<T>(angle1, Angle::HalfPi<T>(), Angle::TwoPi<T>() + Angle::HalfPi<T>()));

	// test when angle is over 2 pi
	CHECK_FLOATING_POINT(T, (T) 0.2, Angle::constrain_angle_two_pi(angle2));
	// test when angle is over pi
	CHECK_FLOATING_POINT(T, (T) 0.3 - Angle::Pi<T>(), Angle::constrain_angle_pi(angle3));

	// test when angle is way over 2 pi
	CHECK_FLOATING_POINT(T, (T) (-0.2) + Angle::TwoPi<T>(), Angle::constrain_angle_two_pi(angle4));

	// test when angle is below two pi
	CHECK_FLOATING_POINT(T, (T) 0.1, Angle::constrain_angle_two_pi(angle5));
	// test when angle is below pi
	CHECK_FLOATING_POINT(T, (T) (-0.21) + Angle::Pi<T>(), Angle::constrain_angle_pi(angle6));

	// test when angle is way under two pi
	CHECK_FLOATING_POINT(T, (T) 0.1, Angle::constrain_angle_pi(angle7));

	// test when function should throw error
	try {
		Angle::constrain_angle<T>(angle1, -1.0, 6.0);
		FAIL();
	} catch (const std::invalid_argument &e) {}

	try {
		Angle::constrain_angle<T>(angle1, Angle::Pi<T>(), -Angle::Pi<T>());
		FAIL();
	} catch (const std::invalid_argument &e) {}

	try {
		Angle::constrain_angle<T>(angle1, 4.0, 6.0);
		FAIL();
	} catch (const std::invalid_argument &e) {}
}

template<class T>
void TestAngleDist() {

	// testing simplest case
	T angle1 = 0.1;
	T angle2 = 0.2;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0.1);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0.1);

	// test when angles are negative
	angle1 = -0.1;
	angle2 = -0.2;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0.1);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0.1);

	// test when one angle is positive, one is negative
	angle1 = -0.1;
	angle2 = 0.2;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0.3);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0.3);

	// test when angles are the same
	angle1 = 1.4;
	angle2 = 1.4;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0);
	angle1 = -2.1;
	angle2 = -2.1;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0);

	// test when angles are around two pi
	angle1 = Angle::TwoPi<T>() - 0.05;
	angle2 = Angle::TwoPi<T>() + 0.05;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0.1);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0.1);
	angle1 = Angle::TwoPi<T>() - 0.05;
	angle2 = 0.05;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0.1);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0.1);

	// test when angles are way above or below two pi
	angle1 = 4 * Angle::TwoPi<T>() + 0.5;
	angle2 = 6 * Angle::TwoPi<T>() + 0.82;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0.32);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0.32);
	angle1 = 4 * Angle::TwoPi<T>() + 0.5;
	angle2 = -6 * Angle::TwoPi<T>() + 0.82;
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle1, angle2), (T) 0.32);
	CHECK_FLOATING_POINT(T, Angle::angle_dist<T>(angle2, angle1), (T) 0.32);
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

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

