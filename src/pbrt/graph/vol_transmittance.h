#pragma once

#include <utility>

#include <pbrt/lights.h>
#include <pbrt/pbrt.h>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolTransmittance {
public:
    VolTransmittance(UniformGraph* boundary, util::MediumData* mediumData, Sampler sampler)
        : boundary(boundary), mediumData(mediumData), sampler(std::move(sampler)) {}

    [[nodiscard]] FreeGraph* CaptureTransmittance(const std::vector<Light>& lights, float amount);

private:
    void GetLitSurfacePoints(std::vector<Vertex*>* litSurfacePoints, Vector3f lightDir);
    void TracePath(const Vertex* surfacePoint, FreeGraph* pathGraph, Vector3f lightDir);

    UniformGraph* boundary;
    util::MediumData* mediumData;
    Sampler sampler;
    float maxDepth = 100;
};

}
