#include "graph_integrator.h"

#include <fstream>
#include <iostream>

#include <pbrt/materials.h>
#include <pbrt/graph/T_sampler_custom.h>
#include <pbrt/util/display.h>
#include <pbrt/util/file.h>
#include <pbrt/util/progressreporter.h>
#include <pbrt/util/string.h>

#include <regex>

#include "deps/nanoflann.hpp"

namespace graph {

using namespace pbrt;

STAT_COUNTER("Integrator/Volume interactions", volumeInteractions)
STAT_COUNTER("Integrator/Surface interactions", surfaceInteractions)
STAT_PERCENT("Integrator/Regularized BSDFs", regularizedBSDFs, totalBSDFs)
STAT_COUNTER("Integrator/Camera rays traced", nCameraRays)

// GraphIntegrator Method Definitions

void GraphIntegrator::Initialize() {
    std::string graphName;
    if (Options->graph.configFile) {
        graphName = std::regex_replace(Options->graph.configFile.value(), std::regex("\\.json"), ".txt");
    } else {
        graphName = std::regex_replace(Options->graph.sceneFile, std::regex("\\.pbrt"), ".txt");
    }
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

        for (Point2i pixel : camera.GetFilm().PixelBounds())
            contributionCache[pixel] = {};
    }
}

void GraphIntegrator::EvaluatePixelSample(Point2i pPixel, int sampleIndex, Sampler sampler,
                                        ScratchBuffer &scratchBuffer) {
    // Initialize _CameraSample_ for current sample
    Filter filter = camera.GetFilm().GetFilter();
    CameraSample cameraSample = GetCameraSample(sampler, pPixel, filter);

    // Generate camera ray for current sample
    pstd::optional<CameraRayDifferential> cameraRay =
            camera.GenerateRayDifferential(cameraSample, mediumData.defaultLambda);

    // Trace _cameraRay_ if valid
    float lightScalar = 0;
    VisibleSurface visibleSurface;
    if (cameraRay) {
        // Double check that the ray's direction is normalized.
        DCHECK_GT(Length(cameraRay->ray.d), .999f);
        DCHECK_LT(Length(cameraRay->ray.d), 1.001f);
        // Scale camera ray differentials based on image sampling rate
        Float rayDiffScale =
                std::max<Float>(.125f, 1 / std::sqrt(static_cast<Float>(sampler.SamplesPerPixel())));
        if (!Options->disablePixelJitter)
            cameraRay->ray.ScaleDifferentials(rayDiffScale);

        ++nCameraRays;
        // Evaluate radiance along camera ray
        lightScalar = Li(cameraRay->ray, pPixel, sampler);

        // Issue warning if unexpected radiance value is returned
        if (IsNaN(lightScalar)) {
            LOG_ERROR("Not-a-number radiance value returned for pixel (%d, %d), sample %d. Setting to black.",
                      pPixel.x, pPixel.y, sampleIndex);
            lightScalar = 0;
        } else if (IsInf(lightScalar)) {
            LOG_ERROR("Infinite radiance value returned for pixel (%d, %d), sample %d. Setting to black.",
                      pPixel.x, pPixel.y, sampleIndex);
            lightScalar = 0;
        }

        PBRT_DBG(
                "%s\n",
                StringPrintf("Camera sample: %s -> ray %s -> L = %s, visibleSurface %s",
                             cameraSample, cameraRay->ray, L, (visibleSurface ? visibleSurface.ToString() : "(none)"))
                             .c_str());
    } else {
        PBRT_DBG("%s\n",
                 StringPrintf("Camera sample: %s -> no ray generated", cameraSample)
                 .c_str());
    }

    int lambdaSamples = 64;

    for (int i = 0; i < lambdaSamples; ++i) {
        SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(sampler.Get1D());
        SampledSpectrum L = lightSpectrum.Sample(lambda) * cameraRay->weight * lightScalar;

        // Add camera ray's contribution to image
        camera.GetFilm().AddSample(pPixel, L, lambda, &visibleSurface, cameraSample.filterWeight);
    }
}

float GraphIntegrator::Li(RayDifferential ray, Point2i pixel, const Sampler& sampler) {
    util::HitsResult mediumHits = GetHits(mediumData.primitiveData.primitive, ray, mediumData);

    if (mediumHits.type == util::OutsideZeroHits || mediumHits.type == util::OutsideOneHit)
        return 0;

    float mediumEntry = mediumHits.type == util::OutsideTwoHits ? mediumHits.tHits[0] : 0;
    float mediumExit = mediumHits.type == util::OutsideTwoHits ? mediumHits.tHits[1] : mediumHits.tHits[0];

    float distBetweenPoints = 2 * graphRadius;

    float L = 0;
    float curT = 0;

    if (mediumHits.type == util::OutsideTwoHits) {
        ShapeIntersection mediumIntersect = mediumHits.intersections[0];
        mediumIntersect.intr.SkipIntersection(&ray, mediumIntersect.tHit);

        curT = mediumHits.tHits[0];
    }

    auto GetNearPointContribution = [&](Point3f searchPoint) -> float {
        searchPoint = worldFromRender(searchPoint);
        Float searchPointArray[3] = {searchPoint.x, searchPoint.y, searchPoint.z};

        std::vector<nanoflann::ResultItem<int, float>> resultItems;

        searchTree->radiusSearch(searchPointArray, squaredRenderRadius, resultItems);

        util::Averager averager;
        for (auto resultItem : resultItems) {
            RefConst<Vertex> foundVertex =  freeGraph->GetVertexConst(vHolder.GetListConst()[resultItem.first].first).value();
            averager.AddValue(foundVertex.get().data.lightScalar); // , 1.f / DistanceSquared(foundVertex.get().point, searchPoint));
        }
        return averager.GetAverage();
    };

    bool cacheFilled = !contributionCache[pixel].empty();
    std::vector<float>& cache = contributionCache[pixel];

    int i = 0;
    while (true) {
        bool stop = false;
        float curDistance = std::max(0.f, curT - mediumEntry);

        if (!cacheFilled)
            cache.push_back(GetNearPointContribution(ray(curDistance)));

        float stepLight = SampleTransmittance(ray, curDistance, sampler, mediumData) * cache[i];

        float distRemaining = mediumExit - curT;
        if (distRemaining > distBetweenPoints) {
            curT += distBetweenPoints;
        }
        else {
            float stepSizeMultiplier = distRemaining / distBetweenPoints;
            stepLight *= stepSizeMultiplier;
            stop = true;
        }

        L += stepLight;
        ++i;
        if (stop)
            break;
    }

    L *= Inv4Pi;
    return L;
}


SampledSpectrum GraphIntegrator::Li(RayDifferential ray, SampledWavelengths& lambda,
                                           Sampler sampler, ScratchBuffer& scratchBuffer,
                                           VisibleSurface* visibleSurface) const {
    ErrorExit("Method unused");
}

std::unique_ptr<GraphIntegrator> GraphIntegrator::Create(
        const ParameterDictionary& parameters, Camera camera, Sampler sampler,
        Primitive aggregate, std::vector<Light> lights)
{
    float graphRadiusMod = parameters.GetOneFloat("graphRadiusMod", 10);
    float renderRadiusMod = parameters.GetOneFloat("renderRadiusMod", 20);
    return std::make_unique<GraphIntegrator>(std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights),
        graphRadiusMod, renderRadiusMod);
}

}
