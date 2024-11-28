#pragma once

#include <pbrt/pbrt.h>

#include <regex>

#include "graph.h"
#include "deps/nanoflann.hpp"
#include "pbrt/cpu/integrators.h"

namespace graph {

using namespace pbrt;

using namespace nanoflann;
using TreeType = KDTreeSingleIndexAdaptor<
    L2_Simple_Adaptor<Float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

class GraphIntegrator final : public RayIntegrator {
public:
    // VolPathCustomIntegrator Public Methods
    GraphIntegrator(int maxDepth, Camera camera, Sampler sampler, Primitive aggregate, std::vector<Light> lights)
            : RayIntegrator(std::move(camera), std::move(sampler), std::move(aggregate), lights),
            maxDepth(maxDepth) {

        light = util::GetLight(lights);
        worldFromRender = camera.GetCameraTransform().WorldFromRender();
        renderFromWorld = camera.GetCameraTransform().RenderFromWorld();
        lightSpectrum = light->GetLEmit();
        defaultLambda = camera.GetFilm().SampleWavelengths(0);
    }

    void Render() override;

    void EvaluatePixelSample(Point2i pPixel, int sampleIndex, Sampler sampler,
                             ScratchBuffer &scratchBuffer) override;

    SampledSpectrum Li(RayDifferential ray, SampledWavelengths &lambda, Sampler sampler,
                       ScratchBuffer &scratchBuffer,
                       VisibleSurface *visibleSurface) const override;

    static std::unique_ptr<GraphIntegrator> Create(
            const ParameterDictionary &parameters, Camera camera, Sampler sampler,
            Primitive aggregate, std::vector<Light> lights);

    [[nodiscard]] float SampleDirectLight(const MediumInteraction &interaction, Sampler sampler) const;

    [[nodiscard]] float ConnectToGraph(const MediumInteraction& connectInteraction, const MediumInteraction& searchInteraction, Sampler sampler) const;

    [[nodiscard]] std::string ToString() const override { return "Graph Integrator"; }

private:
    int maxDepth;
    DistantLight* light;
    Transform worldFromRender;
    Transform renderFromWorld;
    DenselySampledSpectrum lightSpectrum;
    SampledWavelengths defaultLambda;

    std::optional<UniformGraph> uniformGraph;
    std::vector<Bounds3f> voxelBounds;

    std::optional<FreeGraph> freeGraph;
    std::unique_ptr<TreeType> searchTree;
    util::VerticesHolder vHolder;
};

}