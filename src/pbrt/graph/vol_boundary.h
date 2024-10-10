#pragma once

#include <pbrt/pbrt.h>

#include <unordered_set>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolBoundary {
public:
    explicit VolBoundary(const util::MediumData& mediumData) : mediumData(mediumData) {}

    [[nodiscard]] UniformGraph CaptureBoundary(int wantedVertices, int equatorStepSize);
    [[nodiscard]] UniformGraph CaptureBoundary(float spacing, int equatorStepSize);

    [[nodiscard]] UniformGraph FillInside(UniformGraph& boundary);

private:
    [[nodiscard]] Bounds3i GetBounds(const UniformGraph& boundary);
    [[nodiscard]] FreeGraph CaptureBoundary(int equatorStepSize) const;
    void ToSingleLayerAndSaveCast(UniformGraph& boundary);

    const util::MediumData& mediumData;
    std::unordered_map<int, std::unordered_set<Point3i, util::PointHash>> castCache;
};

}
