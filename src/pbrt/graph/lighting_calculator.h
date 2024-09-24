#pragma once

#include <pbrt/base/sampler.h>

#include "graph.h"
#include "pbrt/graph/deps/Eigen/SparseCore"

namespace graph {

using SparseMat = Eigen::SparseMatrix<SampledSpectrum>;
using SparseVec = Eigen::SparseVector<SampledSpectrum>;

class LightingCalculator {
public:
    LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                       DistantLight* light, Sampler sampler, const std::vector<RefConst<Vertex>>& litVertices);
    [[nodiscard]] UniformGraph GetFinalLightGrid(int initialLightingIterations, int transmittanceIterations);

private:
    [[nodiscard]] bool HasSequentialIds() const;
    [[nodiscard]] SparseVec GetLightVector(int initialLightingIterations);
    [[nodiscard]] SparseMat GetTransmittanceMatrix() const;
    [[nodiscard]] SparseMat GetGMatrix() const;
    [[nodiscard]] SparseMat GetPhaseMatrix() const;

    const UniformGraph& grid;
    const util::MediumData& mediumData;
    DistantLight* light;
    Sampler sampler;
    std::vector<RefConst<Vertex>> litVertices;
    Vector3f lightDir;
    int numVertices;
};

}