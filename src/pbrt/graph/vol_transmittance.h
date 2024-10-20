#pragma once

#include <pbrt/lights.h>
#include <pbrt/pbrt.h>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolTransmittance {
public:
    VolTransmittance(const UniformGraph& boundary, const util::MediumData& mediumData, Sampler sampler)
        : boundary(boundary), mediumData(mediumData), sampler(std::move(sampler)) {}

    void CaptureTransmittance(UniformGraph& grid, float sphereStepDegrees, int spheresPerDimension);

private:
    void TraceTransmittancePath(Point3f startPoint, Vector3f direction, UniformGraph& grid);
    [[nodiscard]] float Transmittance(const MediumInteraction& p0, const MediumInteraction& p1);

    const UniformGraph& boundary;
    const util::MediumData& mediumData;
    Sampler sampler;
    int maxDepth = 100;
    int numPathsScatteredOutsideGrid = 0;
};

}
