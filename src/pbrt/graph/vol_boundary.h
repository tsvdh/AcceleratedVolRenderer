#pragma once

#include <pbrt/pbrt.h>
#include <pbrt/util/vecmath.h>
#include <pbrt/cpu/primitive.h>
#include "graph.h"
#include "pbrt/cpu/aggregates.h"

namespace graph {

using namespace pbrt;

class VolBoundary {
public:
    explicit VolBoundary(util::MediumData* mediumData) : mediumData(mediumData) {}

    [[nodiscard]] UniformGraph* CaptureBoundary(float graphSpacing, int horizontalStep, int verticalStep) const;

    void ToSingleLayer(UniformGraph* boundary) const;

private:
    util::MediumData* mediumData;
};

}
