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

class GraphIntegrator : public RayIntegrator {
public:
    // VolPathCustomIntegrator Public Methods
    GraphIntegrator(Camera camera, Sampler sampler, Primitive aggregate, std::vector<Light> lights)
        : RayIntegrator(std::move(camera), std::move(sampler), std::move(aggregate), lights) {
        light = util::GetLight(lights);
        worldFromRender = camera.GetCameraTransform().WorldFromRender();
        renderFromWorld = camera.GetCameraTransform().RenderFromWorld();
        lightSpectrum = light->GetLEmit();
        mediumData = util::MediumData(aggregate, camera.GetFilm().SampleWavelengths(0));

        Initialize();
    }

    void Initialize();

    SampledSpectrum Li(RayDifferential ray, SampledWavelengths &lambda, Sampler sampler,
                       ScratchBuffer &scratchBuffer,
                       VisibleSurface *visibleSurface) const override;

    static std::unique_ptr<GraphIntegrator> Create(
            const ParameterDictionary &parameters, Camera camera, Sampler sampler,
            Primitive aggregate, std::vector<Light> lights);

    [[nodiscard]] float ConnectToGraph(Point3f searchPoint) const;

    [[nodiscard]] std::string ToString() const override { return "Graph Integrator"; }

protected:
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
    float squaredVertexRadius = -1;
    std::vector<float> squaredSearchRanges;
    float maxSquaredSearchRange = -1;
    float perc99SquaredSearchRange = -1;
};

}