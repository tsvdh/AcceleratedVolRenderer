#pragma once
#include <iostream>
#include <ostream>

#include "pbrt/util/print.h"

int main(int argc, char* argv[]) {
    std::cout << pbrt::StringPrintf("%d", 1000) << std::endl;
}
