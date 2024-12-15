#pragma once

#include <pbrt/base/sampler.h>

#include "graph.h"
#include "pbrt/graph/deps/Eigen/SparseCore"

namespace graph {

using SparseMat = Eigen::SparseMatrix<float>;
using SparseVec = Eigen::SparseVector<float>;

class LightingCalculator {
public:
    virtual ~LightingCalculator() = default;

    LightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
        LightingCalculatorConfig config, bool quiet, bool runInParallel);

    [[nodiscard]] virtual SparseVec GetLightVector() = 0;
    void ComputeFinalLight();
    void ComputeFinalLight(const SparseVec& light);

protected:
    void CheckSequentialIds() const;

    [[nodiscard]] virtual SparseMat GetTransmittanceMatrix() const;
    [[nodiscard]] virtual SparseMat GetGMatrix() const = 0;
    [[nodiscard]] SparseMat GetPhaseMatrix() const;
    [[nodiscard]] SparseVec LightMapToVector(const std::unordered_map<int, float>& lightMap) const;

    Graph& graph;
    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    int numVertices;
    LightingCalculatorConfig config;
    bool quiet;
    bool runInParallel;
};

}