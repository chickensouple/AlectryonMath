//
// Created by clark on 4/14/17.
//

#ifndef ALECTYONMATH_TESTHELPERS_H
#define ALECTYONMATH_TESTHELPERS_H

#define CHECK_FLOATING_POINT(T, val1, val2) \
    if (typeid(T) == typeid(float)) ASSERT_NEAR(val1, val2, 1e-5); \
    else ASSERT_NEAR(val1, val2, 1e-13);

#define CHECK_FLOATING_ARR(T, N, arr1, arr2) \
    for (int i = 0; i < N; i++) CHECK_FLOATING_POINT(T, arr1(i), arr2(i));

#endif //ALECTYONMATH_TESTHELPERS_H
