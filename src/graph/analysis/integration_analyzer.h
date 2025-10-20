#pragma once

#include <pbrt/pbrt.h>

#include "graph/graph.h"
#include "graph/graph_integrator.h"
#include "graph/deps/nanoflann.hpp"

namespace graph {

using namespace pbrt;

using StaticTreeType = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<Float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

class IntegrationAnalyzer final : public GraphIntegrator {
public:
    IntegrationAnalyzer(int maxDepth, Camera camera, Sampler sampler, Primitive aggregate, std::vector<Light> lights)
        : GraphIntegrator(std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights)), maxDepth(maxDepth) {
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
    int searchScatters = 0;
    util::Averager rangeAverager;
};

}
