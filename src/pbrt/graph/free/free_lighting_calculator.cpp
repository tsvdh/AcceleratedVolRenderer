#include "free_lighting_calculator.h"

#include "pbrt/lights.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeLightingCalculator::FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler,
        int initialLightingIterations)
    : LightingCalculator(graph, mediumData, light, std::move(sampler), initialLightingIterations) {

    freeGraph = dynamic_cast<FreeGraph*>(&graph);
}

SparseVec FreeLightingCalculator::GetLightVector() {
    int numEntryVertices = 0;
    for (auto& pair : graph.GetVertices()) {
        if (pair.second.data.type && pair.second.data.type == entry)
            ++numEntryVertices;
    }

    std::unordered_map<int, float> lightMap;
    lightMap.reserve(numEntryVertices);

    int workNeeded = numEntryVertices * initialLightingIterations;
    ProgressReporter progress(workNeeded, "Computing initial lighting", false);

    for (auto& [id, vertex] : graph.GetVertices()) {
        if (!vertex.data.type || vertex.data.type != entry)
            continue;

        Point3f graphPoint = vertex.point;
        MediumInteraction interaction(graphPoint, Vector3f(), 0, mediumData.medium, nullptr);

        Ray rayToLight(graphPoint, -lightDir);
        pstd::optional<ShapeIntersection> shapeIntersect = mediumData.aggregate->Intersect(rayToLight, Infinity);
        if (!shapeIntersect) {
            lightMap[id] = 1;
            progress.Update(initialLightingIterations);
            continue;
        }

        Point3f boundaryPoint = shapeIntersect->intr.p();

        lightMap[id] = 0;
        for (int i = 0; i < initialLightingIterations; ++i) {
            sampler.StartPixelSample(Point2i(id, id), i);
            lightMap[id] += Transmittance(interaction, boundaryPoint, mediumData.defaultLambda, sampler);
            progress.Update();
        }
        lightMap[id] /= static_cast<float>(initialLightingIterations);
    }
    progress.Done();

    return LightMapToVector(lightMap);
}

SparseMat FreeLightingCalculator::GetGMatrix() const {
    auto& vertices = graph.GetVerticesConst();
    auto& edges = graph.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> gEntries;
    gEntries.reserve(edges.size());

    for (auto& edge : edges) {
        const Vertex& from = vertices.at(edge.second.from);
        const Vertex& to = vertices.at(edge.second.to);

        float T = edge.second.data.throughput;
        float G = std::min(1 / LengthSquared(from.point - to.point), 1.f);

        gEntries.emplace_back(to.id, from.id, T * G);
    }

    SparseMat gMatrix(numVertices, numVertices);
    gMatrix.setFromTriplets(gEntries.begin(), gEntries.end());
    return gMatrix;
}

SparseMat FreeLightingCalculator::GetTransmittanceMatrix() const {
    std::vector<Eigen::Triplet<float>> weightEntries;
    weightEntries.reserve(numVertices);

    for (int i = 0; i < numVertices; ++i) {
        int numEdges = static_cast<int>(graph.GetVertex(i)->get().inEdges.size());
        float weight = 1 / static_cast<float>(numEdges);

        if (numEdges > 0)
            weightEntries.emplace_back(i, i, weight);
    }

    SparseMat weightMatrix(numVertices, numVertices);
    weightMatrix.setFromTriplets(weightEntries.begin(), weightEntries.end());

    return weightMatrix * LightingCalculator::GetTransmittanceMatrix();
}

}