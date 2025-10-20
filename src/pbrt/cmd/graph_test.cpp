#pragma once
#include <iostream>
#include <ostream>

#include "pbrt/util/print.h"

int main(int argc, char* argv[]) {
    std::vector bla{1, 2, 3, 0, -1, 0, 4, -10, 5};
    bla.erase(std::remove_if(bla.begin(), bla.end(), [](int el) {
        return el <= 0;
    }), bla.end());
    for (int el : bla) {
        std::cout << " " << el;
    }
    std::cout << std::endl;
}
