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

}