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
        lightScalar = Li(cameraRay->ray, sampler);

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

float GraphIntegrator::Li(RayDifferential ray, Sampler sampler) {
    util::HitsResult mediumHits = GetHits(mediumData.primitiveData.primitive, ray, mediumData);

    float distInMedium;
    switch (mediumHits.type) {
        case util::OutsideZeroHits:
            [[fallthrough]];
        case util::OutsideOneHit:
            return 0;
        case util::OutsideTwoHits:
            distInMedium = mediumHits.tHits[1] - mediumHits.tHits[0];
            break;
        case util::InsideOneHit:
            distInMedium = mediumHits.tHits[0];
            break;
        default:
            ErrorExit("Forgot an enum case");
    }

    float mediumEntry = mediumHits.type == util::OutsideTwoHits ? mediumHits.tHits[0] : 0;
    float mediumExit = mediumHits.type == util::OutsideTwoHits ? mediumHits.tHits[1] : mediumHits.tHits[0];

    float sphereRadius = freeGraph->GetVertexRadius().value();
    float distBetweenPoints = 2 * sphereRadius;

    int numSearchPoints = std::ceil(distInMedium / distBetweenPoints);

    std::vector<Point3f> searchPoints;
    searchPoints.reserve(numSearchPoints);

    if (distBetweenPoints / 2 > distInMedium) {
        searchPoints.push_back(ray(mediumEntry));
    } else {
        float curDist = mediumHits.tHits[0] - distBetweenPoints / 2;
        for (int i = 0; i < numSearchPoints; ++i) {
            curDist += distBetweenPoints;
            searchPoints.push_back(ray(curDist));
        }
    }

    float squaredNearbyDistance = Sqr(sphereRadius * Sqrt2);
    std::vector<RefConst<Vertex>> closeVertices;

    for (Point3f searchPoint : searchPoints) {
        searchPoint = worldFromRender(searchPoint);
        Float searchPointArray[3] = {searchPoint.x, searchPoint.y, searchPoint.z};

        std::vector<nanoflann::ResultItem<int, float>> resultItems;

        searchTree->radiusSearch(searchPointArray, squaredNearbyDistance, resultItems);

        for (auto resultItem : resultItems) {
            RefConst<Vertex> foundVertex =  freeGraph->GetVertexConst(vHolder.GetListConst()[resultItem.first].first).value();
            closeVertices.push_back(foundVertex);
        }
    }

    struct LightDistance {
        float entry, exit;
        float light;

        LightDistance(float entry, float exit, float light) : entry(entry), exit(exit), light(light) {}
    };

    util::SphereMaker sphereMaker(sphereRadius);
    std::vector<LightDistance> lightDistances;
    lightDistances.reserve(closeVertices.size());

    for (RefConst<Vertex> vertex : closeVertices) {
        Point3f renderSpacePoint = renderFromWorld(vertex.get().point);
        SphereContainer currentSphere = sphereMaker.GetSphereFor(renderSpacePoint);

        float light = vertex.get().data.lightScalar;
        util::HitsResult sphereHits = GetHits(currentSphere.sphere, ray, mediumData);

        switch (sphereHits.type) {
            case util::OutsideZeroHits:
                [[fallthrough]];
            case util::OutsideOneHit:
                break;
            case util::OutsideTwoHits:
                lightDistances.emplace_back(sphereHits.tHits[0], sphereHits.tHits[1], light);
                break;
            case util::InsideOneHit:
                lightDistances.emplace_back(0, sphereHits.tHits[0], light);
                break;
            default:
                ErrorExit("Forgot an enum case");
        }
    }

    std::sort(lightDistances.begin(), lightDistances.end(),
        [](const LightDistance& a, const LightDistance& b) { return a.entry < b.entry; });

    float L = 0;
    float curTr = 1;
    float curT = 0;

    if (mediumHits.type == util::OutsideTwoHits) {
        ShapeIntersection mediumIntersect = mediumHits.intersections[0];
        mediumIntersect.intr.SkipIntersection(&ray, mediumIntersect.tHit);

        curT = mediumHits.tHits[0];
    }

    for (const LightDistance& lightDistance : lightDistances) {
        if ((lightDistance.entry < mediumEntry && lightDistance.exit < mediumEntry) || lightDistance.entry > mediumExit)
            continue;

        float stepDist = std::max(0.f, lightDistance.entry - curT);

        if (stepDist > 0) {
            float stepTr = 0;
            for (int i = 0; i < stepIterations; ++i)
                stepTr += SampleTransmittance(ray, stepDist, sampler, mediumData);

            stepTr /= static_cast<float>(stepIterations);
            curTr *= stepTr;

            ray.o = ray(stepDist);
            curT += stepDist;
        }

        L += lightDistance.light;
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
    int stepIterations = parameters.GetOneInt("stepIterations", 20);
    int renderRadiusMod = parameters.GetOneInt("renderRadiusMod", 10);
    return std::make_unique<GraphIntegrator>(std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights),
        stepIterations, renderRadiusMod);
}

}
