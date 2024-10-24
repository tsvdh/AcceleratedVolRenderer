#include "lighting_calculator.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
LightingCalculator::LightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler)
    : graph(graph), mediumData(mediumData), light(light), sampler(std::move(sampler)) {

    numVertices = static_cast<int>(graph.GetVerticesConst().size());
}

void LightingCalculator::ComputeFinalLight(int transmittanceIterations) {
    SparseVec light = GetLightVector();

    if (transmittanceIterations > 0) {
        SparseMat transmittance = GetTransmittanceMatrix();
        SparseVec curLight = light;

        ProgressReporter progress(transmittanceIterations, "Computing final lighting", false);

        for (int i = 0; i < transmittanceIterations; ++i) {
            curLight = transmittance * curLight;
            light += curLight;
            progress.Update();
        }
        progress.Done();
    }

    for (auto& pair : graph.GetVertices()) {
        Vertex& v = pair.second;
        v.data.lightScalar = light.coeffRef(v.id);
    }
}

SparseMat LightingCalculator::GetPhaseMatrix() const {
    std::vector<Eigen::Triplet<float>> phaseEntries;
    phaseEntries.reserve(numVertices);

    for (int i = 0; i < numVertices; ++i)
        phaseEntries.emplace_back(i, i, Inv4Pi);

    SparseMat phaseMatrix(numVertices, numVertices);
    phaseMatrix.setFromTriplets(phaseEntries.begin(), phaseEntries.end());
    return phaseMatrix;
}

SparseMat LightingCalculator::GetGMatrix() const {
    auto& vertices = graph.GetVerticesConst();
    auto& edges = graph.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> gEntries;
    gEntries.reserve(edges.size());

    for (auto& edge : edges) {
        const Vertex& from = vertices.at(edge.second.from);
        const Vertex& to = vertices.at(edge.second.to);

        float T = edge.second.data.throughput;
        double edgeLength = std::pow(Length(from.point - to.point), 2);
        auto G = static_cast<float>(1 / edgeLength);

        gEntries.emplace_back(to.id, from.id, T * G);
    }

    SparseMat gMatrix(numVertices, numVertices);
    gMatrix.setFromTriplets(gEntries.begin(), gEntries.end());
    return gMatrix;
}

SparseMat LightingCalculator::GetTransmittanceMatrix() const {
    return GetPhaseMatrix() * GetGMatrix();
}

SparseVec LightingCalculator::LightMapToVector(const std::unordered_map<int, float>& lightMap) const {
    std::vector<std::pair<int, float>> lightPairs(lightMap.begin(), lightMap.end());
    std::sort(lightPairs.begin(), lightPairs.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    SparseVec lightVector(numVertices);
    for (auto& pair : lightPairs) {
        lightVector.coeffRef(pair.first) = pair.second;
    }

    return lightVector;
}

}