#pragma once

#include <pbrt/base/sampler.h>

#include "graph.h"
#include "pbrt/graph/deps/Eigen/Dense"

namespace graph {

using Eigen::Matrix;
using Eigen::Dynamic;

class LightingCalculator {
public:
    LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                       DistantLight* light, Sampler sampler, const std::vector<RefConst<Vertex>>& litVertices);
    [[nodiscard]] UniformGraph GetFinalLightGrid(int initialLightingIterations, int transmittanceIterations);

private:
    [[nodiscard]] bool HasSequentialIds() const;
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, 1> GetLightVector(int initialLightingIterations);
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, Dynamic> GetTransmittanceMatrix() const;
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, Dynamic> GetGMatrix() const;
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, Dynamic> GetPhaseMatrix() const;

    const UniformGraph& grid;
    const util::MediumData& mediumData;
    DistantLight* light;
    Sampler sampler;
    std::vector<RefConst<Vertex>> litVertices;
    Vector3f lightDir;
    int numVertices;
};

}