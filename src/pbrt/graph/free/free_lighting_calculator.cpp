#include "free_lighting_calculator.h"

#include <iostream>

namespace graph {

FreeLightingCalculator::FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler)
    : LightingCalculator(graph, mediumData, light, std::move(sampler)) {

    freeGraph = dynamic_cast<FreeGraph*>(&graph);
}

SparseVec FreeLightingCalculator::GetLightVector() {
    if (graph.GetPaths().empty())
        std::cerr << "No vertices to add light to" << std::endl;

    std::unordered_map<int, float> lightMap;
    lightMap.reserve(graph.GetPaths().size());

    for (auto& pair : graph.GetPaths()) {
        lightMap[pair.second.edges[0]] = 1;
    }

    return LightMapToVector(lightMap);
}

}