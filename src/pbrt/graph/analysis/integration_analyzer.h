#pragma once

#include <pbrt/pbrt.h>

#include "pbrt/graph/graph.h"
#include "pbrt/graph/graph_integrator.h"
#include "pbrt/graph/deps/nanoflann.hpp"

namespace graph {

using namespace pbrt;

using StaticTreeType = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<Float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

class IntegrationAnalyzer final : public GraphIntegrator {
public:
    IntegrationAnalyzer(float renderRadiusMod, float neighbourRadiusMod, int maxDepth, Camera camera, Sampler sampler, Primitive aggregate, std::vector<Light> lights)
        : GraphIntegrator(renderRadiusMod, neighbourRadiusMod, std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights)), maxDepth(maxDepth) {
    }

    void Render() override;

    void AnalyzeRay(RayDifferential ray, Sampler sampler, const SampledWavelengths& lambda);

    static std::unique_ptr<IntegrationAnalyzer> Create(
            const ParameterDictionary &parameters, Camera camera, Sampler sampler,
            Primitive aggregate, std::vector<Light> lights);

    [[nodiscard]] std::string ToString() const override { return "Integration Analyzer"; }

private:
    int maxDepth;

    int totalScatters = 0;
    int nodeScatters = 0;
};

}
