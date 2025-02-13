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

void LightingCalculator::ComputeFinalLight(int bouncesIndex) {
    SparseVec initialLight = GetLightVector();
    ComputeFinalLight(initialLight, bouncesIndex);
}

void LightingCalculator::ComputeFinalLight(const SparseVec& light, int bouncesIndex) {
    int numVertices = static_cast<int>(graph.GetVertices().size());
    int bounces = config.bounces[bouncesIndex];

    SparseVec finalLight(light);
    SparseVec finalWeights(numVertices);
    for (int i = 0; i < numVertices; ++i)
        finalWeights.coeffRef(i) = 1.f;

    if (bounces > 0) {
        SparseMat transmittance = GetTransmittanceMatrix();
        SparseMat connections = GetConnectionMatrix();

        SparseVec curLight = finalLight;
        SparseVec curWeights = finalWeights;

        ProgressReporter progress(bounces, "Computing final lighting", quiet);

        for (int iteration = 0; iteration < bounces; ++iteration) {
            curLight = transmittance * curLight;
            curWeights = connections * curWeights;

            SparseVec invCurWeights(numVertices);
            for (int i = 0; i < numVertices; ++i) {
                float weight = curWeights.coeff(i);
                invCurWeights.coeffRef(i) = weight == 0.f ? 0.f : 1.f / weight;
            }

            finalLight += curLight.cwiseProduct(invCurWeights);

            progress.Update();
        }
        progress.Done();
    }

    for (auto& [id, vertex] : graph.GetVertices())
        vertex.data.lightScalar = finalLight.coeff(id);
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

SparseMat LightingCalculator::GetConnectionMatrix() const {
    auto& edges = graph.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> connectionEntries;
    connectionEntries.reserve(edges.size());

    for (auto& edge : edges)
        connectionEntries.emplace_back(edge.second.to, edge.second.from, 1);

    SparseMat connectionMatrix(numVertices, numVertices);
    connectionMatrix.setFromTriplets(connectionEntries.begin(), connectionEntries.end());
    return connectionMatrix;
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
