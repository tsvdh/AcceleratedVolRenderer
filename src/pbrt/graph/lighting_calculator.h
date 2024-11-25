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

    LightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler,
        int initialLightingIterations);
    void ComputeFinalLight(int transmittanceIterations);

protected:
    void CheckSequentialIds() const;
    [[nodiscard]] virtual SparseVec GetLightVector() = 0;
    [[nodiscard]] SparseMat GetTransmittanceMatrix() const;
    [[nodiscard]] virtual SparseMat GetGMatrix() const = 0;
    [[nodiscard]] SparseMat GetPhaseMatrix() const;
    [[nodiscard]] SparseVec LightMapToVector(const std::unordered_map<int, float>& lightMap) const;

    Graph& graph;
    const util::MediumData& mediumData;
    DistantLight* light;
    Vector3f lightDir;
    Sampler sampler;
    int numVertices;
    int initialLightingIterations;
};

}