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

    void CaptureTransmittance(UniformGraph& grid, int multiplier);

private:
    void TraceTransmittancePath(const Vertex& gridPoint, UniformGraph& grid);
    [[nodiscard]] SampledSpectrum Transmittance(const MediumInteraction& p0, const MediumInteraction& p1) const;

    const UniformGraph& boundary;
    const util::MediumData& mediumData;
    DistantLight* light;
    Vector3f lightDir;
    Sampler sampler;
    int maxDepth = 100;
    int numPathsScatteredOutsideGrid = 0;
};

}
