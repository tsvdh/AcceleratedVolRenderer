#include "graph_integrator.h"

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
void GraphIntegrator::Render() {
    // Handle debugStart, if set
    if (!Options->debugStart.empty()) {
        std::vector<int> c = SplitStringToInts(Options->debugStart, ',');
        if (c.empty())
            ErrorExit("Didn't find integer values after --debugstart: %s",
                      Options->debugStart);
        if (c.size() != 3)
            ErrorExit("Didn't find three integer values after --debugstart: %s",
                      Options->debugStart);

        Point2i pPixel(c[0], c[1]);
        int sampleIndex = c[2];

        ScratchBuffer scratchBuffer(65536);
        Sampler tileSampler = samplerPrototype.Clone(Allocator());
        tileSampler.StartPixelSample(pPixel, sampleIndex);

        EvaluatePixelSample(pPixel, sampleIndex, tileSampler, scratchBuffer);

        return;
    }

    thread_local Point2i threadPixel;
    thread_local int threadSampleIndex;
    CheckCallbackScope _([&]() {
        return StringPrintf("Rendering failed at pixel (%d, %d) sample %d. Debug with "
                            "\"--debugstart %d,%d,%d\"\n",
                            threadPixel.x, threadPixel.y, threadSampleIndex,
                            threadPixel.x, threadPixel.y, threadSampleIndex);
    });

    if (Options->graphDebug) {
        ProgressReporter progress(static_cast<int>(sceneGrid.GetVertices().size()), "Preprocessing voxels", false);

        Vector3f voxelHalfDiagonal = Vector3f(1, 1, 1) * sceneGrid.GetSpacing() / 2;

        for (auto& pair : sceneGrid.GetVertices()) {
            if (pair.second.data.lightScalar == 0)
                continue;

            Point3f voxelMiddle = pair.second.point;
            voxelBounds.emplace_back(voxelMiddle - voxelHalfDiagonal, voxelMiddle + voxelHalfDiagonal);
        }
    }
    else {
        // ProgressReporter progress(static_cast<int>(sceneGrid.GetVerticesConst().size()), "Preprocessing spectral data", false);
        //
        // SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(0);
        // pstd::span<const float> lambdaSpan(&lambda[0], NSpectrumSamples);
        //
        // for (auto& pair : sceneGrid.GetVertices()) {
        //     Vertex& vertex = pair.second;
        //     vertex.data.continuousLighting = PiecewiseLinearSpectrum(lambdaSpan, vertex.data.lighting->GetValues());
        //     vertex.data.lighting = std::nullopt;
        //     progress.Update();
        // }
        // progress.Done();
    }

    // Declare common variables for rendering image in tiles
    ThreadLocal<ScratchBuffer> scratchBuffers([]() { return ScratchBuffer(); });

    ThreadLocal<Sampler> samplers([this]() { return samplerPrototype.Clone(); });

    Bounds2i pixelBounds = camera.GetFilm().PixelBounds();
    int spp = Options->graphDebug ? 8 : 256;
    ProgressReporter progress(static_cast<int64_t>(spp) * pixelBounds.Area(), "Rendering", Options->quiet);

    int waveStart = 0, waveEnd = 1, nextWaveSize = 1;

    if (Options->recordPixelStatistics)
        StatsEnablePixelStats(pixelBounds,
                              RemoveExtension(camera.GetFilm().GetFilename()));
    // Handle MSE reference image, if provided
    pstd::optional<Image> referenceImage;
    FILE *mseOutFile = nullptr;
    if (!Options->mseReferenceImage.empty()) {
        auto [image, metadata] = Image::Read(Options->mseReferenceImage);
        referenceImage = image;

        Bounds2i msePixelBounds =
                metadata.pixelBounds
                ? *metadata.pixelBounds
                : Bounds2i(Point2i(0, 0), referenceImage->Resolution());
        if (!Inside(pixelBounds, msePixelBounds))
            ErrorExit("Output image pixel bounds %s aren't inside the MSE "
                      "image's pixel bounds %s.",
                      pixelBounds, msePixelBounds);

        // Transform the pixelBounds of the image we're rendering to the
        // coordinate system with msePixelBounds.pMin at the origin, which
        // in turn gives us the section of the MSE image to crop. (This is
        // complicated by the fact that Image doesn't support pixel
        // bounds...)
        Bounds2i cropBounds(Point2i(pixelBounds.pMin - msePixelBounds.pMin),
                            Point2i(pixelBounds.pMax - msePixelBounds.pMin));
        *referenceImage = referenceImage->Crop(cropBounds);
        CHECK_EQ(referenceImage->Resolution(), Point2i(pixelBounds.Diagonal()));

        mseOutFile = FOpenWrite(Options->mseReferenceOutput);
        if (!mseOutFile)
            ErrorExit("%s: %s", Options->mseReferenceOutput, ErrorString());
    }

    // Connect to display server if needed
    if (!Options->displayServer.empty()) {
        Film film = camera.GetFilm();
        DisplayDynamic(film.GetFilename(), Point2i(pixelBounds.Diagonal()),
                       {"R", "G", "B"},
                       [&](Bounds2i b, pstd::span<pstd::span<float>> displayValue) {
                           int index = 0;
                           for (Point2i p : b) {
                               RGB rgb = film.GetPixelRGB(pixelBounds.pMin + p,
                                                          2.f / static_cast<float>(waveStart + waveEnd));
                               for (int c = 0; c < 3; ++c)
                                   displayValue[c][index] = rgb[c];
                               ++index;
                           }
                       });
    }

    // Render image in waves
    while (waveStart < spp) {

        // // Render current wave's image pixels in series
        // for (int x = pixelBounds.pMin.x; x < pixelBounds.pMax.x; x++) {
        //     for (int y = pixelBounds.pMin.y; y < pixelBounds.pMax.y; y++) {
        //         Point2i pPixel(x, y);
        //         ScratchBuffer& scratchBuffer = scratchBuffers.Get();
        //         Sampler& sampler = samplers.Get();
        //
        //         StatsReportPixelStart(pPixel);
        //         threadPixel = pPixel;
        //         // Render samples in pixel _pPixel_
        //         for (int sampleIndex = waveStart; sampleIndex < waveEnd; ++sampleIndex) {
        //             threadSampleIndex = sampleIndex;
        //             sampler.StartPixelSample(pPixel, sampleIndex);
        //             EvaluatePixelSample(pPixel, sampleIndex, sampler, scratchBuffer);
        //             scratchBuffer.Reset();
        //             progress.Update(1);
        //         }
        //
        //         StatsReportPixelEnd(pPixel);
        //     }
        // }

        // Render current wave's image tiles in parallel
        ParallelFor2D(pixelBounds, [&](Bounds2i tileBounds) {
            // Render image tile given by _tileBounds_
            ScratchBuffer &scratchBuffer = scratchBuffers.Get();
            Sampler &sampler = samplers.Get();

            for (Point2i pPixel : tileBounds) {
                StatsReportPixelStart(pPixel);
                threadPixel = pPixel;
                // Render samples in pixel _pPixel_
                for (int sampleIndex = waveStart; sampleIndex < waveEnd; ++sampleIndex) {
                    threadSampleIndex = sampleIndex;
                    sampler.StartPixelSample(pPixel, sampleIndex);
                    EvaluatePixelSample(pPixel, sampleIndex, sampler, scratchBuffer);
                    scratchBuffer.Reset();
                }

                StatsReportPixelEnd(pPixel);
            }
            progress.Update((waveEnd - waveStart) * tileBounds.Area());
        });

        // Update start and end wave
        waveStart = waveEnd;
        waveEnd = std::min(spp, waveEnd + nextWaveSize);
        if (!referenceImage)
            nextWaveSize = std::min(2 * nextWaveSize, 64);
        if (waveStart == spp)
            progress.Done();

        // Optionally write current image to disk
        if (waveStart == spp || Options->writePartialImages || referenceImage) {
            LOG_VERBOSE("Writing image with spp = %d", waveStart);
            ImageMetadata metadata;
            metadata.renderTimeSeconds = static_cast<float>(progress.ElapsedSeconds());
            metadata.samplesPerPixel = waveStart;
            if (referenceImage) {
                ImageMetadata filmMetadata;
                Image filmImage =
                        camera.GetFilm().GetImage(&filmMetadata, 1.f / static_cast<float>(waveStart));
                ImageChannelValues mse =
                        filmImage.MSE(filmImage.AllChannelsDesc(), *referenceImage);
                fprintf(mseOutFile, "%d, %.9g\n", waveStart, mse.Average());
                metadata.MSE = mse.Average();
                fflush(mseOutFile);
            }
            if (waveStart == spp || Options->writePartialImages) {
                camera.InitMetadata(&metadata);
                camera.GetFilm().WriteImage(metadata, 1.f / static_cast<float>(waveStart));
            }
        }
    }

    if (mseOutFile)
        fclose(mseOutFile);
    DisconnectFromDisplayServer();
    LOG_VERBOSE("Rendering finished");
}

void GraphIntegrator::EvaluatePixelSample(Point2i pPixel, int sampleIndex, Sampler sampler,
                                        ScratchBuffer &scratchBuffer) {
    // Sample wavelengths for the ray
    Float lu = sampler.Get1D();
    SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(lu);

    // Initialize _CameraSample_ for current sample
    Filter filter = camera.GetFilm().GetFilter();
    CameraSample cameraSample = GetCameraSample(sampler, pPixel, filter);

    // Generate camera ray for current sample
    pstd::optional<CameraRayDifferential> cameraRay =
            camera.GenerateRayDifferential(cameraSample, lambda);

    // Trace _cameraRay_ if valid
    SampledSpectrum L(0.);
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
        bool initializeVisibleSurface = camera.GetFilm().UsesVisibleSurface();

        L = cameraRay->weight * Li(cameraRay->ray, lambda, sampler, scratchBuffer,
                                   initializeVisibleSurface ? &visibleSurface : nullptr);

        // Issue warning if unexpected radiance value is returned
        if (L.HasNaNs()) {
            LOG_ERROR("Not-a-number radiance value returned for pixel (%d, "
                      "%d), sample %d. Setting to black.",
                      pPixel.x, pPixel.y, sampleIndex);
            L = SampledSpectrum(0.f);
        } else if (IsInf(L.y(lambda))) {
            LOG_ERROR("Infinite radiance value returned for pixel (%d, %d), "
                      "sample %d. Setting to black.",
                      pPixel.x, pPixel.y, sampleIndex);
            L = SampledSpectrum(0.f);
        }

        PBRT_DBG(
                "%s\n",
                StringPrintf("Camera sample: %s -> ray %s -> L = %s, visibleSurface %s",
                             cameraSample, cameraRay->ray, L,
                             (visibleSurface ? visibleSurface.ToString() : "(none)"))
                        .c_str());
    } else {
        PBRT_DBG("%s\n",
                 StringPrintf("Camera sample: %s -> no ray generated", cameraSample)
                         .c_str());
    }
    // Add camera ray's contribution to image
    camera.GetFilm().AddSample(pPixel, L, lambda, &visibleSurface,
                               cameraSample.filterWeight);
}

SampledSpectrum GraphIntegrator::Li(RayDifferential ray, SampledWavelengths& lambda,
                                           Sampler sampler, ScratchBuffer& scratchBuffer,
                                           VisibleSurface* visibleSurface) const {
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

    if (Options->graphDebug) {
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
            if (OptRefConst<Vertex> optVertex = sceneGrid.GetVertexConst(middle)) {
                float scalar = optVertex.value().get().data.lightScalar;
                return lightSpectrum.Sample(lambda) * scalar;
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
                optVertex = sceneGrid.GetVertexConst(p);
                return false;
            }
            else {
                // Handle null scattering along ray path
                return true;
            }
        });

    if (optVertex) {
        float scalar = optVertex.value().get().data.lightScalar;
        return lightSpectrum.Sample(lambda) * scalar * Inv4Pi;
    }

    return SampledSpectrum(0);
}

SampledSpectrum GraphIntegrator::SampleLd(const Interaction& intr, const BSDF* bsdf,
                                            SampledWavelengths& lambda, Sampler sampler,
                                            SampledSpectrum beta,
                                            SampledSpectrum r_p) const {
    return SampledSpectrum(0);
}

std::string GraphIntegrator::ToString() const {
    return StringPrintf(
            "[ GraphIntegrator maxDepth: %d lightSampler: %s regularize: %s ]", maxDepth,
            lightSampler, regularize);
}

std::unique_ptr<GraphIntegrator> GraphIntegrator::Create(
        const ParameterDictionary& parameters, Camera camera, Sampler sampler,
        Primitive aggregate, std::vector<Light> lights)
{
    int maxDepth = parameters.GetOneInt("maxdepth", 5);
    std::string lightStrategy = parameters.GetOneString("lightsampler", "bvh");
    bool regularize = parameters.GetOneBool("regularize", false);

    return std::make_unique<GraphIntegrator>(
            maxDepth, std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights),
            lightStrategy, regularize);
}

}
