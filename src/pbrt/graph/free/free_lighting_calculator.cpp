#include "free_lighting_calculator.h"

#include <iostream>

namespace graph {

FreeLightingCalculator::FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler)
    : LightingCalculator(graph, mediumData, light, std::move(sampler)) {

    freeGraph = dynamic_cast<FreeGraph*>(&graph);
}

SparseVec FreeLightingCalculator::GetLightVector() {
    std::unordered_map<int, float> lightMap;
    lightMap.reserve(graph.GetPaths().size());

    for (auto& pair : graph.GetVertices()) {
        if (pair.second.data.type && pair.second.data.type == entry)
        lightMap[pair.first] = 1;
    }

    return LightMapToVector(lightMap);
}

}