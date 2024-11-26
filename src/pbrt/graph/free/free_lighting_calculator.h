#pragma once
#include "pbrt/graph/lighting_calculator.h"

namespace graph {

class FreeLightingCalculator final : public LightingCalculator {
public:
    FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler,
        int initialLightingIterations);
    
private:
    [[nodiscard]] SparseMat GetGMatrix() const override;
    [[nodiscard]] SparseVec GetLightVector() override;
    [[nodiscard]] SparseMat GetTransmittanceMatrix() const override;

    FreeGraph* freeGraph;
};

}