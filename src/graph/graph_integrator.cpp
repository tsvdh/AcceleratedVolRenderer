#include "graph_integrator.h"

#include <fstream>
#include <iostream>

#include <graph/T_sampler_custom.h>
#include <pbrt/materials.h>
#include <pbrt/util/display.h>
#include <pbrt/util/file.h>
#include <pbrt/util/progressreporter.h>
#include <pbrt/util/string.h>

#include <regex>

#include "deps/nanoflann.hpp"

namespace graph {

STAT_COUNTER("Integrator/Volume interactions", volumeInteractions)
STAT_COUNTER("Integrator/Surface interactions", surfaceInteractions)
STAT_PERCENT("Integrator/Regularized BSDFs", regularizedBSDFs, totalBSDFs)
STAT_COUNTER("Integrator/Camera rays traced", nCameraRays)

// GraphIntegrator Method Definitions
void GraphIntegrator::Initialize() {
    std::string graphName = Options->graph.dataFile.has_value()
        ?  Options->graph.dataFile.value()
        : std::regex_replace(Options->graph.sceneFile, std::regex("\\.pbrt"), ".txt");

    std::ifstream file(util::FileNameToPath(graphName));
    std::string description, name;
    file >> description >> name;

    if (name == "uniform")
        uniformGraph = UniformGraph::ReadFromDisk(graphName);
    if (name == "free")
        freeGraph = FreeGraph::ReadFromDisk(graphName);

    if (uniformGraph) {
        if (Options->graph.debug) {
            ProgressReporter progress(static_cast<int>(uniformGraph->GetVertices().size()), "Preprocessing voxels", false);

            Vector3f voxelHalfDiagonal = Vector3f(1, 1, 1) * uniformGraph->GetSpacing() / 2;

            for (auto& pair : uniformGraph->GetVertices()) {
                if (pair.second.data.lightScalar == 0)
                    continue;

                Point3f voxelMiddle = pair.second.point;
                voxelBounds.emplace_back(voxelMiddle - voxelHalfDiagonal, voxelMiddle + voxelHalfDiagonal);
            }
        }
    }
    if (freeGraph) {
        std::cout << "Building kd-tree... ";
        vHolder = freeGraph->GetVerticesList();
        searchTree = std::make_unique<StaticTreeType>(3, vHolder);
        std::cout << "done" << std::endl;
    }
}


SampledSpectrum GraphIntegrator::Li(RayDifferential ray, SampledWavelengths& lambda,
                                           Sampler sampler, ScratchBuffer& scratchBuffer,
                                           VisibleSurface* visibleSurface) const {
    SampledSpectrum spectrum = lightSpectrum.Sample(lambda);

    if (uniformGraph) {
        pstd::optional<ShapeIntersection> shapeIsect = Intersect(ray);
        if (!shapeIsect)
            return SampledSpectrum(0);

        if (!ray.medium) {
            shapeIsect->intr.SkipIntersection(&ray, shapeIsect->tHit);
            shapeIsect = Intersect(ray);
        }

        if (!ray.medium)
            return SampledSpectrum(0);

        float tMax = shapeIsect ? shapeIsect->tHit : Infinity;

        if (Options->graph.debug) {
            auto hit0 = std::make_unique<float>(-1);
            auto hit1 = std::make_unique<float>(-1);

            float minHit0 = Infinity;
            float minHit1 = Infinity;

            RayDifferential worldRay = worldFromRender(ray);

            for (const Bounds3f& voxelBounds : voxelBounds) {
                if (voxelBounds.IntersectP(worldRay.o, worldRay.d, Infinity, hit0.get(), hit1.get())) {
                    if (*hit0 < minHit0) {
                        minHit0 = *hit0;
                        minHit1 = *hit1;
                    }
                }
            }

            if (minHit0 != Infinity) {
                Point3f middle = worldRay((minHit0 + minHit1) / 2);
                if (OptRefConst<Vertex> optVertex = uniformGraph->GetVertexConst(middle)) {
                    float lightScalar = optVertex.value().get().data.lightScalar;
                    return lightSpectrum.Sample(lambda) * lightScalar;
                }
            }

            return SampledSpectrum(0);
        }

        // Initialize _RNG_ for sampling the majorant transmittance
        uint64_t hash0 = Hash(sampler.Get1D());
        uint64_t hash1 = Hash(sampler.Get1D());
        RNG rng(hash0, hash1);

        OptRefConst<Vertex> optVertex;

        // Sample new point on ray
        SampleT_maj(
            static_cast<Ray&>(ray), tMax, sampler.Get1D(), rng, lambda,
            [&](Point3f p, MediumProperties mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {
                // Handle medium scattering event for ray

                // Compute medium event probabilities for interaction
                Float pAbsorb = mp.sigma_a[0] / sigma_maj[0];
                Float pScatter = mp.sigma_s[0] / sigma_maj[0];
                Float pNull = std::max<Float>(0, 1 - pAbsorb - pScatter);

                CHECK_GE(1 - pAbsorb - pScatter, -1e-6);
                // Sample medium scattering event type and update path
                Float um = rng.Uniform<Float>();
                int mode = SampleDiscrete({pAbsorb, pScatter, pNull}, um);

                if (mode == 0) {
                    // Handle absorption along ray path
                    return false;
                }
                if (mode == 1) {
                    // Handle scattering along ray path
                    p = worldFromRender(p);
                    optVertex = uniformGraph->GetVertexConst(p);
                    return false;
                }
                else {
                    // Handle null scattering along ray path
                    return true;
                }
            });

        if (optVertex) {
            float lightScalar = optVertex.value().get().data.lightScalar;
            return spectrum * lightScalar * Inv4Pi;
        }

        return SampledSpectrum(0);
    }

    if (freeGraph) {
        float L = 0;

        // support for camera inside the volume
        // if (pstd::optional<ShapeIntersection> shapeIntersection = Intersect(ray)) {
        //     ray.medium = shapeIntersection->intr.mediumInterface->inside;
        // }

        while (true) {
            pstd::optional<ShapeIntersection> shapeIntersection = Intersect(ray);
            if (ray.medium) {
                // Sample the participating medium
                bool terminated = false;
                Float tMax = shapeIntersection ? shapeIntersection->tHit : Infinity;

                // Initialize _RNG_ for sampling the majorant transmittance
                uint64_t hash0 = Hash(sampler.Get1D());
                uint64_t hash1 = Hash(sampler.Get1D());
                RNG rng(hash0, hash1);

                // Sample new point on ray
                SampleT_maj(static_cast<Ray&>(ray), tMax, sampler.Get1D(), rng, lambda,
                    [&](Point3f p, MediumProperties mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {
                        // Handle medium scattering event for ray

                        // Compute medium event probabilities for interaction
                        Float pAbsorb = mp.sigma_a[0] / sigma_maj[0];
                        Float pScatter = mp.sigma_s[0] / sigma_maj[0];
                        Float pNull = std::max<Float>(0, 1 - pAbsorb - pScatter);

                        CHECK_GE(1 - pAbsorb - pScatter, -1e-6);
                        // Sample medium scattering event type and update path
                        Float um = rng.Uniform<Float>();
                        int mode = SampleDiscrete({pAbsorb, pScatter, pNull}, um);

                        if (mode == 0) {
                            // Handle absorption along ray path
                            terminated = true;
                            return false;
                        }
                        if (mode == 1) {
                            // Handle scattering along ray path
                            L += ConnectToGraph(p);
                            terminated = true;
                            return false;
                        }
                        else {
                            // Handle null scattering along ray path
                            return true;
                        }
                    });

                if (terminated) {
                    break;
                }
            }

            if (!shapeIntersection)
                break;

            shapeIntersection->intr.SkipIntersection(&ray, shapeIntersection->tHit);
        }

        return spectrum * L;
    }

    ErrorExit("free graph or uniform graph should be non null");
}

float GraphIntegrator::ConnectToGraph(Point3f searchPoint) const {
    searchPoint = worldFromRender(searchPoint);
    Float searchPointArray[3] = {searchPoint.x, searchPoint.y, searchPoint.z};

    std::vector<nanoflann::ResultItem<int, float>> resultItems;

    searchTree->radiusSearch(searchPointArray, squaredSearchRadius, resultItems);

    if (resultItems.empty() && squaredNeighbourSearchRadius != -1)
        searchTree->radiusSearch(searchPointArray, squaredNeighbourSearchRadius, resultItems);

    util::Averager lightAverager(static_cast<int>(resultItems.size()));

    for (nanoflann::ResultItem resultItem : resultItems) {
        const Vertex& v = freeGraph->GetVertexConst(resultItem.first)->get();
        lightAverager.AddValue(v.data.lightScalar, 1.f / resultItem.second); // squared distance
    }

    return lightAverager.GetAverage();
}

std::unique_ptr<GraphIntegrator> GraphIntegrator::Create(
        const ParameterDictionary& parameters, Camera camera, Sampler sampler,
        Primitive aggregate, std::vector<Light> lights)
{
    float renderRadiusMod = parameters.GetOneFloat("renderRadiusMod", 1);
    float neighbourRadiusMod = parameters.GetOneFloat("neighbourRadiusMod", 1);

    return std::make_unique<GraphIntegrator>(renderRadiusMod, neighbourRadiusMod,
        std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights));
}

}
