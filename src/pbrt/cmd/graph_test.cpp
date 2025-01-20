#pragma once

#include <iostream>

#include "pbrt/util/vecmath.h"

void main(int argc, char* argv[]) {
    pbrt::Point3f a;
    pbrt::Point3f b{};
    std::cout << a << b << std::endl;
}
