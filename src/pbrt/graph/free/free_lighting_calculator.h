#pragma once
#include "pbrt/graph/lighting_calculator.h"

namespace graph {

class FreeLightingCalculator final : public LightingCalculator {
public:
    FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler);
    
private:
    [[nodiscard]] SparseVec GetLightVector() override;

    FreeGraph* freeGraph;
    Vector3f lightDir;
};

}