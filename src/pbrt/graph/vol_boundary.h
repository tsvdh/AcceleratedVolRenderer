#pragma once

#include <pbrt/pbrt.h>
#include <pbrt/util/vecmath.h>
#include "graph.h"

namespace graph {

using namespace pbrt;

class VolBoundary {
public:
    VolBoundary(Medium& medium, Point3f center)
        : medium(medium), center(center) {}

    void CaptureBoundary(UniformGraph& graph, int horizontalStep, int verticalStep);

private:
    Medium& medium;
    Point3f center;
};

}
