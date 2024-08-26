#pragma once

#include <pbrt/pbrt.h>

#include "graph.h"
#include "pbrt/cpu/integrators.h"

namespace graph {

using namespace pbrt;

class GraphIntegrator final : public RayIntegrator {
public:
    // VolPathCustomIntegrator Public Methods
    GraphIntegrator(int maxDepth, Camera camera, Sampler sampler, Primitive aggregate,
                           std::vector<Light> lights,
                           const std::string &lightSampleStrategy = "bvh",
                           bool regularize = false)
            : RayIntegrator(std::move(camera), std::move(sampler), std::move(aggregate), lights),
            maxDepth(maxDepth),
            lightSampler(LightSampler::Create(lightSampleStrategy, lights, Allocator())),
            regularize(regularize) {}

    void Render() override;

    void EvaluatePixelSample(Point2i pPixel, int sampleIndex, Sampler sampler,
                             ScratchBuffer &scratchBuffer) override;

    SampledSpectrum Li(RayDifferential ray, SampledWavelengths &lambda, Sampler sampler,
                       ScratchBuffer &scratchBuffer,
                       VisibleSurface *visibleSurface) const;

    static std::unique_ptr<GraphIntegrator> Create(
            const ParameterDictionary &parameters, Camera camera, Sampler sampler,
            Primitive aggregate, std::vector<Light> lights);

    [[nodiscard]] std::string ToString() const override;

private:
    // VolPathCustomIntegrator Private Methods
    SampledSpectrum SampleLd(const Interaction &intr, const BSDF *bsdf,
                             SampledWavelengths &lambda, Sampler sampler,
                             SampledSpectrum beta, SampledSpectrum r_p) const;

    // VolPathCustomIntegrator Private Members
    int maxDepth;
    LightSampler lightSampler;
    bool regularize;
    FreeGraph pathGraph;
};

}