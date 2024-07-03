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

    [[nodiscard]] FreeGraph* CaptureTransmittance(std::vector<Light> lights);

private:
    void GetLitSurfacePoints(std::vector<Vertex*>* litSurfacePoints, Vector3f lightDir);
    void TracePath(Vertex* surfacePoint, FreeGraph* pathGraph, Vector3f lightDir);

    UniformGraph* boundary;
    util::MediumData* mediumData;
    Sampler sampler;
    float maxDepth = 100;
};

}
