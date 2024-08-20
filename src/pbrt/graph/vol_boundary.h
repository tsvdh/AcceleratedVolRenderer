#pragma once

#include <pbrt/pbrt.h>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolBoundary {
public:
    explicit VolBoundary(util::MediumData* mediumData) : mediumData(mediumData) {}

    [[nodiscard]] UniformGraph* CaptureBoundary(int wantedVertices, int horizontalStep, int verticalStep) const;
    [[nodiscard]] UniformGraph* CaptureBoundary(float spacing, int horizontalStep, int verticalStep) const;

    void ToSingleLayer(UniformGraph* boundary) const;

private:
    [[nodiscard]] FreeGraph CaptureBoundary(int horizontalStep, int verticalStep) const;

    util::MediumData* mediumData;
};

}
