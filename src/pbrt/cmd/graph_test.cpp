#pragma once
#include <iostream>
#include <ostream>

#include "graph/deps/json.hpp"
#include "pbrt/util/print.h"

int main(int argc, char* argv[]) {
    int a = 1000000;
    int b = 1000000;
    int c = a * b;
    int64_t d = a * b;
    std::cout << c << " " << d << std::endl;
}
