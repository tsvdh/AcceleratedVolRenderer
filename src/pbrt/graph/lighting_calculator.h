#pragma once

#include <pbrt/base/sampler.h>

#include "graph.h"
#include "pbrt/graph/deps/Eigen/SparseCore"

namespace graph {

using SparseMat = Eigen::SparseMatrix<float>;
using SparseVec = Eigen::SparseVector<float>;

class LightingCalculator {
public:
    LightingCalculator(FreeGraph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
        const LightingCalculatorConfig& config, bool quiet);

    [[nodiscard]] SparseVec GetLightVector();
    [[nodiscard]] SparseMat GetTransportMatrix() const;
    int ComputeFinalLight(int bouncesIndex = 0);
    int ComputeFinalLight(const SparseVec& lightVec, const SparseMat& transportMat, int bouncesIndex = 0);

private:
    [[nodiscard]] SparseVec LightMapToVector(const std::unordered_map<int, float>& lightMap) const;

    FreeGraph& graph;
    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    int numVertices;
    LightingCalculatorConfig config;
    bool quiet;
};

}