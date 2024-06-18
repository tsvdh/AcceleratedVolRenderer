#pragma once

#include <pbrt/pbrt.h>
#include <pbrt/util/vecmath.h>
#include <pbrt/cpu/primitive.h>
#include "graph.h"

namespace graph {

using namespace pbrt;

class VolBoundary {
public:
    explicit VolBoundary(Primitive& accel, SampledWavelengths lambda);

    UniformGraph* CaptureBoundary(float graphSpacing, int horizontalStep, int verticalStep);

private:
    SampledWavelengths lambda;
    Medium medium;
    Point3f boundsCenter;
    float maxDistToCenter;
};

}
