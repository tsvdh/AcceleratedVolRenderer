#include "free_lighting_calculator.h"

#include "pbrt/lights.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeLightingCalculator::FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler,
        int initialLightingIterations)
    : LightingCalculator(graph, mediumData, light, std::move(sampler)), initialLightingIterations(initialLightingIterations) {

    lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));
    freeGraph = dynamic_cast<FreeGraph*>(&graph);
}

SparseVec FreeLightingCalculator::GetLightVector() {
    std::unordered_map<int, float> lightMap;
    lightMap.reserve(graph.GetPaths().size());

    int workNeeded = static_cast<int>(graph.GetVertices().size()) * initialLightingIterations;
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