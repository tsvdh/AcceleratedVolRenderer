#pragma once

#include "graph.h"
#include "pbrt/graph/deps/Eigen/Dense"

namespace graph {

using Eigen::Matrix;
using Eigen::Dynamic;

class LightingCalculator {
public:
    LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                       DistantLight* light, const std::vector<RefConst<Vertex>>& litVertices);
    [[nodiscard]] UniformGraph GetFinalLightGrid(int numIterations) const;

private:
    [[nodiscard]] bool HasSequentialIds() const;
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, 1> GetLightVector() const;
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, Dynamic> GetTransmittanceMatrix() const;
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, Dynamic> GetGMatrix() const;
    [[nodiscard]] Matrix<SampledSpectrum, Dynamic, Dynamic> GetPhaseMatrix() const;

    const UniformGraph& grid;
    const util::MediumData& mediumData;
    DistantLight* light;
    std::vector<RefConst<Vertex>> litVertices;
    int numVertices;
};

}