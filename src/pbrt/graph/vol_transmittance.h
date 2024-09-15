#pragma once

#include <pbrt/lights.h>
#include <pbrt/pbrt.h>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolTransmittance {
public:
    VolTransmittance(const UniformGraph& boundary, const util::MediumData& mediumData,
                     DistantLight* light, Sampler sampler);

    void CaptureTransmittance(UniformGraph& grid, float amount, int multiplier);
    [[nodiscard]] std::vector<RefConst<Vertex>> GetLitSurfacePoints();

private:
    void TracePath(const Vertex& surfacePoint, UniformGraph& grid);
    [[nodiscard]] SampledSpectrum Transmittance(const MediumInteraction& p0, const MediumInteraction& p1) const;

    const UniformGraph& boundary;
    const util::MediumData& mediumData;
    DistantLight* light;
    Vector3f lightDir;
    Sampler sampler;
    int maxDepth = 100;
};

}
