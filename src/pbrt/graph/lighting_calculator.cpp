#include "lighting_calculator.h"

#include "pbrt/lights.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
LightingCalculator::LightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
                                       LightingCalculatorConfig config, bool quiet, int sampleIndexOffset)
    : graph(graph), mediumData(mediumData), inDirection(inDirection), sampler(std::move(sampler)), config(config), quiet(quiet),
      sampleIndexOffset(sampleIndexOffset) {
    if (config.lightRayIterations <= 0)
        ErrorExit("Must have at least one light ray iteration");

    CheckSequentialIds();
    numVertices = static_cast<int>(graph.GetVerticesConst().size());
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

void LightingCalculator::ComputeFinalLight() {
    SparseVec initialLight = GetLightVector();
    ComputeFinalLight(initialLight);
}

void LightingCalculator::ComputeFinalLight(const SparseVec& light) {
    SparseVec finalLight(light);

    if (config.transmittanceMatrixIterations > 0) {
        SparseMat transmittance = GetTransmittanceMatrix();
        SparseVec curLight = finalLight;

        ProgressReporter progress(config.transmittanceMatrixIterations, "Computing final lighting", quiet);

        for (int i = 0; i < config.transmittanceMatrixIterations; ++i) {
            curLight = transmittance * curLight;
            finalLight += curLight;
            progress.Update();
        }
        progress.Done();
    }

    for (auto& pair : graph.GetVertices()) {
        Vertex& v = pair.second;
        v.data.lightScalar = finalLight.coeff(v.id);
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
    return GetGMatrix();
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
