#pragma once
#include "pbrt/graph/lighting_calculator.h"

namespace graph {

class FreeLightingCalculator final : public LightingCalculator {
public:
    FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
        LightingCalculatorConfig config, bool quiet, bool runInParallel);

    [[nodiscard]] SparseVec GetLightVector() override;
    
private:
    [[nodiscard]] SparseMat GetGMatrix() const override;
    [[nodiscard]] SparseMat GetTransmittanceMatrix() const override;

    FreeGraph* freeGraph;
};

}