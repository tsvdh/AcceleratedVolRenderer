#include "lighting_calculator.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
LightingCalculator::LightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler,
        int initialLightingIterations)
    : graph(graph), mediumData(mediumData), light(light), sampler(std::move(sampler)), initialLightingIterations(initialLightingIterations) {

    if (initialLightingIterations <= 0)
        ErrorExit("Must have at least one initial lighting iteration");

    CheckSequentialIds();

    numVertices = static_cast<int>(graph.GetVerticesConst().size());
    lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));
}

void LightingCalculator::CheckSequentialIds() const {
    std::vector<int> ids;
    ids.reserve(graph.GetVertices().size());

    for (auto& [id, _] : graph.GetVerticesConst())
        ids.push_back(id);

    std::sort(ids.begin(), ids.end(), [](int a, int b) { return a < b; });

    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] != i)
            ErrorExit("Graph must have sequential vertex ids for mapping to matrices");
    }
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
