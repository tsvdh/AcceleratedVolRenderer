#pragma once

#include <utility>

#include <pbrt/lights.h>
#include <pbrt/pbrt.h>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolTransmittance {
public:
    VolTransmittance(const UniformGraph& boundary, const util::MediumData& mediumData, Sampler sampler)
        : boundary(boundary), mediumData(mediumData), sampler(std::move(sampler)) {}

    [[nodiscard]] FreeGraph CaptureTransmittance(const std::vector<Light>& lights, float amount);

private:
    std::vector<Ref<const Vertex>> GetLitSurfacePoints(Vector3f lightDir);
    void TracePath(const Vertex& surfacePoint, FreeGraph& pathGraph, Vector3f lightDir);

    const UniformGraph& boundary;
    const util::MediumData& mediumData;
    Sampler sampler;
    float maxDepth = 100;
};

}
