#pragma once

#include <iostream>
#include "pbrt/graph/deps/Eigen/SparseCore"

#include "pbrt/util/vecmath.h"

void main(int argc, char* argv[]) {
    Eigen::SparseVector<float> test(3);
    test.coeffRef(0) = 1;
    test.coeffRef(1) = 0.5;
    test.coeffRef(2) = 0;

    Eigen::SparseVector<float> test2(3);
    test2.coeffRef(0) = 1;
    test2.coeffRef(1) = 2;
    test2.coeffRef(2) = 0;

    std::cout << test.cwiseQuotient(test2) << std::endl;
    std::cout << test2.cwiseQuotient(test).real() << std::endl;
}
