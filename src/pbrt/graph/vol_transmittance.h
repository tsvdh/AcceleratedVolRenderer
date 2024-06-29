#pragma once

#include <pbrt/pbrt.h>
#include <pbrt/lights.h>

#include <utility>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolTransmittance {
public:
    VolTransmittance(UniformGraph* boundary, util::MediumData* mediumData, Sampler sampler)
        : boundary(boundary), mediumData(mediumData), sampler(std::move(sampler)) {}

    void TracePath(Vertex* surfacePoint, FreeGraph* pathGraph, Vector3f lightDir);
    [[nodiscard]] FreeGraph* CaptureTransmittance(std::vector<Light> lights);

private:
    UniformGraph* boundary;
    util::MediumData* mediumData;
    Sampler sampler;
    float maxDepth = 100;
};

}
