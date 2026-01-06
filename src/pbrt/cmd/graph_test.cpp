#pragma once
#include <iostream>
#include <ostream>

#include "graph/deps/json.hpp"
#include "pbrt/util/math.h"
#include "pbrt/util/print.h"

int main(int argc, char* argv[]) {
    int64_t a = 1000000;
    int b = 1000000;
    int64_t c = static_cast<int64_t>(pbrt::Sqr(b));
    std::cout << c << std::endl;
}
