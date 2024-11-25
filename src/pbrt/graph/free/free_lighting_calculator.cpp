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

    for (auto& pair : graph.GetVertices()) {
        if (!pair.second.data.type || pair.second.data.type != entry)
            continue;

        Point3f graphPoint = pair.second.point;
        MediumInteraction interaction(graphPoint, Vector3f(), 0, mediumData.medium, nullptr);

        Ray rayToLight(graphPoint, -lightDir);
        Point3f boundaryPoint = mediumData.aggregate->Intersect(rayToLight, Infinity).value().intr.p();

        lightMap[pair.first] = 0;
        for (int i = 0; i < initialLightingIterations; ++i) {
            sampler.StartPixelSample(Point2i(pair.first, pair.first), i);
            lightMap[pair.first] += Transmittance(interaction, boundaryPoint, mediumData.defaultLambda, sampler);
            progress.Update();
        }
        lightMap[pair.first] /= static_cast<float>(initialLightingIterations);
    }
    progress.Done();

    return LightMapToVector(lightMap);
}

SparseMat FreeLightingCalculator::GetGMatrix() const {
    auto& vertices = graph.GetVerticesConst();
    auto& edges = graph.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> gEntries;
    gEntries.reserve(edges.size());

    // std::vector<float> gTerms;
    // gTerms.reserve(edges.size());

    for (auto& edge : edges) {
        const Vertex& from = vertices.at(edge.second.from);
        const Vertex& to = vertices.at(edge.second.to);

        float T = edge.second.data.throughput;
        // float G = std::min(1 / LengthSquared(from.point - to.point), 1.f);

        // gTerms.push_back(G);

        gEntries.emplace_back(to.id, from.id, T * 1);
    }

    // std::sort(gTerms.begin(), gTerms.end(), [](float a, float b) { return a < b; });
    // for (float g : gTerms) {
    //     std::cout << g << std::endl;
    // }

    SparseMat gMatrix(numVertices, numVertices);
    gMatrix.setFromTriplets(gEntries.begin(), gEntries.end());
    return gMatrix;
}


}