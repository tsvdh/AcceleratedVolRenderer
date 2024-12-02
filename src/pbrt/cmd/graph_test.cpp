#pragma once

#include <iostream>

#include <pbrt/graph/graph.h>
#include <pbrt/scene.h>
#include <pbrt/graph/voxels/voxel_transmittance.h>
#include <pbrt/util/args.h>

#include "pbrt/graph/voxels/voxel_boundary.h"

#include <pbrt/graph/deps/nanoflann.hpp>
#include <pbrt/graph/deps/Eigen/Dense>

using namespace pbrt;

void main(int argc, char* argv[]) {
    PBRTOptions options;
    InitPBRT(options);

    std::unordered_map<int, float> bla;
    int size = 1000;
    bla.reserve(size);

    for (int i = 0; i < size; ++i) {
        bla[i] = 0;
    }

    ParallelFor(0, size, [&](int i) {
        bla[i] = 1;
        for (int j = 0; j < i; ++j) {
            bla[i] += 1;
        }
    });

    int x = 0;
}