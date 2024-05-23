#pragma once

#include <pbrt/pbrt.h>
#include <pbrt/cpu/integrators.h>

namespace graph {

using namespace pbrt;

class GraphVolPathIntegrator : public RayIntegrator {
public:
    // GraphVolPathIntegrator Public Methods
    GraphVolPathIntegrator(int maxDepth, Camera camera, Sampler sampler, Primitive aggregate,
                      std::vector<Light> lights,
                      const std::string &lightSampleStrategy = "bvh",
                      bool regularize = false)
            : RayIntegrator(camera, sampler, aggregate, lights),
              maxDepth(maxDepth),
              lightSampler(LightSampler::Create(lightSampleStrategy, lights, Allocator())),
              regularize(regularize) {}

    SampledSpectrum Li(RayDifferential ray, SampledWavelengths &lambda, Sampler sampler,
                       ScratchBuffer &scratchBuffer,
                       VisibleSurface *visibleSurface) const;

    static std::unique_ptr<GraphVolPathIntegrator> Create(
            const ParameterDictionary &parameters, Camera camera, Sampler sampler,
            Primitive aggregate, std::vector<Light> lights, const FileLoc *loc);

    std::string ToString() const;

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
