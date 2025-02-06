#pragma once
#include "pbrt/graph/lighting_calculator.h"

namespace graph {

class FreeLightingCalculator final : public LightingCalculator {
public:
    FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
        LightingCalculatorConfig config, bool quiet, int sampleIndexOffset = 0);

    [[nodiscard]] SparseVec GetLightVector() override;
    
private:
    [[nodiscard]] SparseMat GetGMatrix() const override;

    FreeGraph* freeGraph;
};

}