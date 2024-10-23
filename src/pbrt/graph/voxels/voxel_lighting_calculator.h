#pragma once
#include "pbrt/graph/lighting_calculator.h"

namespace graph {

class VoxelLightingCalculator final : public LightingCalculator {
public:
    VoxelLightingCalculator(Graph& graph, const util::MediumData& mediumData, DistantLight* light, Sampler sampler,
        int initialLightingIterations);

private:
    [[nodiscard]] bool HasSequentialIds() const;
    [[nodiscard]] SparseVec GetLightVector() override;
    [[nodiscard]] SparseMat GetGMatrix() const override;

    UniformGraph* uniformGraph;
    Vector3f lightDir;
    int initialLightingIterations;
};

}