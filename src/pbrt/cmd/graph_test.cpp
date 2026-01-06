#pragma once
#include <iostream>
#include <ostream>

#include "graph/deps/json.hpp"
#include "pbrt/util/print.h"

int main(int argc, char* argv[]) {
    using json = nlohmann::basic_json<std::map, std::vector, std::string, bool, std::int64_t, std::uint64_t, float>
}
