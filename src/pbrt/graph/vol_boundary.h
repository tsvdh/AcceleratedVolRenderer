#pragma once

#include <pbrt/pbrt.h>

#include <unordered_set>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolBoundary {
public:
    explicit VolBoundary(const util::MediumData& mediumData) : mediumData(mediumData) {}

    [[nodiscard]] UniformGraph CaptureBoundary(int wantedVertices, int horizontalStep, int verticalStep);
    [[nodiscard]] UniformGraph CaptureBoundary(float spacing, int horizontalStep, int verticalStep);

    [[nodiscard]] UniformGraph FillInside(UniformGraph& boundary);

private:
    [[nodiscard]] FreeGraph CaptureBoundary(int horizontalStep, int verticalStep) const;
    void ToSingleLayerAndSaveCast(UniformGraph& boundary);

    const util::MediumData& mediumData;
    std::unordered_map<int, std::unordered_set<Point3i, util::PointHash>> castCache;
};

}
