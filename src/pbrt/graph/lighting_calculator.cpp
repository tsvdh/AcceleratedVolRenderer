#include "lighting_calculator.h"

#include "pbrt/lights.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
LightingCalculator::LightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
                                       const LightingCalculatorConfig& config, bool quiet, int sampleIndexOffset)
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

int LightingCalculator::ComputeFinalLight(int bouncesIndex) {
    SparseVec initialLight = GetLightVector();
    return ComputeFinalLight(initialLight, bouncesIndex);
}

int LightingCalculator::ComputeFinalLight(const SparseVec& light, int bouncesIndex) {
    int bounces = config.bounces[bouncesIndex];

    SparseVec finalLight(light);
    SparseVec finalWeights(numVertices);
    for (int i = 0; i < numVertices; ++i)
        finalWeights.coeffRef(i) = 1.f;

    int curIteration = 0;

    if (bounces > 0) {
        SparseMat transmittance = GetTransmittanceMatrix();
        // SparseMat weightedTransmittance = transmittance * GetPathContinueMatrix();
        SparseVec pathContinueVector = GetPathContinueVector();

        SparseVec curLight = finalLight;
        SparseVec curWeights = finalWeights;

        ProgressReporter progress(bounces, "Computing final lighting", quiet);

        for (; curIteration < bounces; ++curIteration) {
            curLight = (transmittance * curLight).cwiseProduct(pathContinueVector);
            curWeights = transmittance * curWeights;

            bool invalidNumbers = false;
            for (int i = 0; i < numVertices; ++i) {
                if (IsNaN(curLight.coeff(i)) || IsInf(curLight.coeff(i))
                    || IsNaN(curWeights.coeff(i)) || IsInf(curWeights.coeff(i))) {
                    invalidNumbers = true;
                    break;
                }
            }
            if (invalidNumbers)
                break;

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

    return curIteration;
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

SparseMat LightingCalculator::GetPathContinueMatrix() const {
    auto& vertices = graph.GetVerticesConst();

    std::vector<Eigen::Triplet<float>> pathContinueEntries;
    pathContinueEntries.reserve(vertices.size());

    for (auto& [id, vertex] : vertices) {
        if (vertex.data.pathContinuePDF.value == -1 && !vertex.outEdges.empty())
            ErrorExit("No continue PDF for outgoing edges");
        pathContinueEntries.emplace_back(id, id, vertex.data.pathContinuePDF.value);
    }

    SparseMat pathContinueMatrix(numVertices, numVertices);
    pathContinueMatrix.setFromTriplets(pathContinueEntries.begin(), pathContinueEntries.end());
    return pathContinueMatrix;
}

SparseVec LightingCalculator::GetPathContinueVector() const {
    SparseVec pathContinueVector(numVertices);

    for (int i = 0; i < graph.GetVertices().size(); ++i)
        pathContinueVector.coeffRef(i) = graph.GetVertex(i)->get().data.pathContinuePDF.value;

    return pathContinueVector;
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
