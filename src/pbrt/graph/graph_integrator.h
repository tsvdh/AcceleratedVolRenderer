#pragma once

#include "graph.h"

#include <pbrt/pbrt.h>
#include <pbrt/cpu/integrators.h>

#include <utility>

namespace graph {

using namespace pbrt;

class GraphVolPathIntegrator : public RayIntegrator {
public:
    // GraphVolPathIntegrator Public Methods
    GraphVolPathIntegrator(int maxDepth, Camera camera, Sampler sampler, Primitive aggregate,
                           std::vector<Light> lights,
                           const std::string &lightSampleStrategy = "bvh",
                           bool regularize = false)
            : RayIntegrator(std::move(camera), std::move(sampler), std::move(aggregate), lights),
            maxDepth(maxDepth),
            lightSampler(LightSampler::Create(lightSampleStrategy, lights, Allocator())),
            regularize(regularize) {}

    void Render() override;

    void EvaluatePixelSample(Point2i pPixel, int sampleIndex, Sampler sampler,
                             ScratchBuffer &scratchBuffer, Graph& graph);

    SampledSpectrum Li(RayDifferential ray, SampledWavelengths &lambda, Sampler sampler,
                       ScratchBuffer &scratchBuffer,
                       VisibleSurface *visibleSurface, Graph& graph) const;

    SampledSpectrum Li(RayDifferential ray, SampledWavelengths &lambda, Sampler sampler,
                       ScratchBuffer &scratchBuffer,
                       VisibleSurface *visibleSurface) const override {
        FreeGraph graph;
        return Li(ray, lambda, sampler, scratchBuffer, visibleSurface, graph);
    }

    static std::unique_ptr<GraphVolPathIntegrator> Create(
            const ParameterDictionary &parameters, Camera camera, Sampler sampler,
            Primitive aggregate, std::vector<Light> lights);



    [[nodiscard]] std::string ToString() const override;

private:
    // GraphVolPathIntegrator Private Methods
    SampledSpectrum SampleLd(const Interaction &intr, const BSDF *bsdf,
                             SampledWavelengths &lambda, Sampler sampler,
                             SampledSpectrum beta, SampledSpectrum inv_w_u) const;

    // GraphVolPathIntegrator Private Members
    int maxDepth;
    LightSampler lightSampler;
    bool regularize;
};

}
