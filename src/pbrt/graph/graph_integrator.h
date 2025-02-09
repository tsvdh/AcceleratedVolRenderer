#pragma once

#include <pbrt/pbrt.h>

#include <regex>

#include "graph.h"
#include "deps/nanoflann.hpp"
#include "pbrt/cpu/integrators.h"

namespace graph {

using namespace pbrt;

using StaticTreeType = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<Float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

class GraphIntegrator final : public RayIntegrator {
public:
    // VolPathCustomIntegrator Public Methods
    GraphIntegrator(Camera camera, Sampler sampler, Primitive aggregate, std::vector<Light> lights,
            int stepIterations, int renderRadiusMod)
        : RayIntegrator(std::move(camera), std::move(sampler), std::move(aggregate), lights),
          stepIterations(stepIterations) {
        light = util::GetLight(lights);
        worldFromRender = camera.GetCameraTransform().WorldFromRender();
        renderFromWorld = camera.GetCameraTransform().RenderFromWorld();
        lightSpectrum = light->GetLEmit();
        mediumData = util::MediumData(aggregate, camera.GetFilm().SampleWavelengths(0));
        squaredSearchRadius = Sqr(GetSameSpotRadius(mediumData) * renderRadiusMod);

        if (!camera.Is<PerspectiveCamera>())
            ErrorExit("Only Perspective camera allowed");

        Initialize();

        if (uniformGraph)
            ErrorExit("Uniform graph no longer supported");

        if (!freeGraph)
            ErrorExit("Free graph is required");

        freeGraph->SetVertexRadius(std::sqrt(squaredSearchRadius));
    }

    void Initialize();

    void EvaluatePixelSample(Point2i pPixel, int sampleIndex, Sampler sampler,
                             ScratchBuffer &scratchBuffer) override;

    SampledSpectrum Li(RayDifferential ray, SampledWavelengths &lambda, Sampler sampler,
                       ScratchBuffer &scratchBuffer,
                       VisibleSurface *visibleSurface) const override;

    [[nodiscard]] float Li(RayDifferential ray, Sampler sampler);

    static std::unique_ptr<GraphIntegrator> Create(
            const ParameterDictionary &parameters, Camera camera, Sampler sampler,
            Primitive aggregate, std::vector<Light> lights);

    [[nodiscard]] std::string ToString() const override { return "Graph Integrator"; }

private:
    DistantLight* light;
    Transform worldFromRender;
    Transform renderFromWorld;
    DenselySampledSpectrum lightSpectrum;
    util::MediumData mediumData;

    std::optional<UniformGraph> uniformGraph;
    std::vector<Bounds3f> voxelBounds;

    std::optional<FreeGraph> freeGraph;
    std::unique_ptr<StaticTreeType> searchTree;
    util::VerticesHolder vHolder;
    float squaredSearchRadius;
    int stepIterations;
};

}