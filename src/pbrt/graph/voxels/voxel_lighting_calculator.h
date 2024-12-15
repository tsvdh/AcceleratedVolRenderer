#pragma once
#include "pbrt/graph/lighting_calculator.h"

namespace graph {

class VoxelLightingCalculator final : public LightingCalculator {
public:
    VoxelLightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
        LightingCalculatorConfig config, bool quiet, bool runInParallel);

    [[nodiscard]] SparseVec GetLightVector() override;

private:
    [[nodiscard]] SparseMat GetGMatrix() const override;

    UniformGraph* uniformGraph;
};

}