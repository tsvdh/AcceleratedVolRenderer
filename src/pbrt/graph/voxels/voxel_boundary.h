#pragma once

#include <pbrt/pbrt.h>

#include <unordered_set>

#include "../graph.h"

namespace graph {

using namespace pbrt;

class VoxelBoundary {
public:
    explicit VoxelBoundary(const util::MediumData& mediumData) : mediumData(mediumData) {}

    [[nodiscard]] UniformGraph CaptureBoundary(int wantedVertices, float equatorStepSize);
    [[nodiscard]] UniformGraph CaptureBoundary(float spacing, float equatorStepSize);

    [[nodiscard]] UniformGraph FillInside(UniformGraph& boundary);

private:
    [[nodiscard]] FreeGraph CaptureBoundary(float equatorStepSize) const;
    void ToSingleLayerAndSaveCast(UniformGraph& boundary);

    const util::MediumData& mediumData;
    std::unordered_map<int, std::unordered_set<Point3i, util::PointHash>> castCache;
};

}
