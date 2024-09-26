#pragma once

#include <pbrt/base/sampler.h>

#include "graph.h"
#include "pbrt/graph/deps/Eigen/SparseCore"

namespace graph {

using SparseMat = Eigen::SparseMatrix<SampledSpectrum>;
using SparseVec = Eigen::SparseVector<SampledSpectrum>;

class LightingCalculator {
public:
    LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData, DistantLight* light, Sampler sampler);
    [[nodiscard]] UniformGraph GetFinalLightGrid(int initialLightingIterations, int lightRaysPerVoxelDist, int transmittanceIterations);

private:
    [[nodiscard]] bool HasSequentialIds() const;
    [[nodiscard]] SparseVec GetLightVector(int initialLightingIterations, int lightRaysPerVoxelDist);
    [[nodiscard]] SparseMat GetTransmittanceMatrix() const;
    [[nodiscard]] SparseMat GetGMatrix() const;
    [[nodiscard]] SparseMat GetPhaseMatrix() const;

    const UniformGraph& grid;
    const util::MediumData& mediumData;
    DistantLight* light;
    Sampler sampler;
    Vector3f lightDir;
    int numVertices;
};

}