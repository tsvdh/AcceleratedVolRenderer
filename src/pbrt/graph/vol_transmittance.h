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

    void CaptureTransmittance(UniformGraph& grid, const std::vector<Light>& lights, float amount, int multiplier);

private:
    std::vector<RefConst<Vertex>> GetLitSurfacePoints(Vector3f lightDir);
    void TracePath(const Vertex& surfacePoint, UniformGraph& grid, Vector3f lightDir);
    [[nodiscard]] SampledSpectrum Transmittance(const MediumInteraction& p0, const MediumInteraction& p1) const;

    const UniformGraph& boundary;
    const util::MediumData& mediumData;
    Sampler sampler;
    int maxDepth = 100;
};

}
