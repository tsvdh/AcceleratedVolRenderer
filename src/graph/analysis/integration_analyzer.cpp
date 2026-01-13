#pragma once

#include "integration_analyzer.h"

namespace graph {

void IntegrationAnalyzer::Render() {
    // {440, 252}, {441, 252}, {727, 395}, {728, 395}, {561, 387}, {562, 387}
    // {500, 389}, {500, 390}, {500, 391}
    // {651, 389}, {533, 340}
    std::vector<Point2i> points{{410, 260}, {736, 396}};
    std::vector maxDepths{1};

    Sampler sampler = samplerPrototype.Clone();

    for (Point2i pixel : points) {
        for (int _maxDepth : maxDepths) {
            this->maxDepth = _maxDepth;

            for (int i = 0; i < sampler.SamplesPerPixel(); ++i) {
                sampler.StartPixelSample(pixel, i);

                Float lu = sampler.Get1D();
                if (Options->disableWavelengthJitter)
                    lu = 0.5;
                SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(lu);

                Filter filter = camera.GetFilm().GetFilter();
                CameraSample cameraSample = GetCameraSample(sampler, pixel, filter);

                CameraRayDifferential cameraRay = camera.GenerateRayDifferential(cameraSample, lambda).value();

                Float rayDiffScale = std::max<Float>(.125f, 1 / std::sqrt(static_cast<Float>(sampler.SamplesPerPixel())));
                if (!Options->disablePixelJitter)
                    cameraRay.ray.ScaleDifferentials(rayDiffScale);

                AnalyzeRay(cameraRay.ray, sampler, lambda);
            }

            std::cout << StringPrintf("%s, %s / %s (%s) | %s / %s (%s), %s", pixel,
                nodeScatters, totalScatters, static_cast<float>(nodeScatters) / static_cast<float>(totalScatters),
                searchScatters, totalScatters, static_cast<float>(searchScatters) / static_cast<float>(totalScatters),
                    rangeAverager.GetAverage()
                ) << std::endl;

            totalScatters = 0;
            nodeScatters = 0;
            searchScatters = 0;
            rangeAverager = util::Averager{};
        }
        std::cout << std::endl;
    }
}

void IntegrationAnalyzer::AnalyzeRay(RayDifferential ray, Sampler sampler, const SampledWavelengths& lambda) {
    auto Analyze = [&](Point3f searchPoint) -> void {
        ++totalScatters;

        Point3f p = worldFromRender(searchPoint);
        Float searchPointArray[3] = {p.x, p.y, p.z};

        std::vector<nanoflann::ResultItem<int, float>> resultItems;

        searchTree->radiusSearch(searchPointArray, squaredVertexRadius, resultItems);

        if (resultItems.size() > 0)
            ++nodeScatters;

        searchTree->radiusSearch(searchPointArray, maxSquaredSearchRange, resultItems);

        resultItems.erase(std::remove_if(resultItems.begin(), resultItems.end(), [&](auto resultItem) {
            const Vertex& vertex = freeGraph->GetVertexConst(resultItem.first)->get();
            return resultItem.second > Sqr(vertex.data.renderSearchRange);
        }), resultItems.end());

        if (resultItems.size() > 0) {
            ++searchScatters;

            for (nanoflann::ResultItem resultItem : resultItems) {
                rangeAverager.AddValue(sqrt(resultItem.second));
            }
        }
    };

    int depth = 0;

    while (true) {
        pstd::optional<ShapeIntersection> shapeIntersection = Intersect(ray);
        if (ray.medium) {
            // Sample the participating medium
            bool scattered = false, terminated = false;
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
                        if (depth++ >= maxDepth) {
                            terminated = true;
                            return false;
                        }

                        Analyze(p);

                        MediumInteraction mediumInteraction(p, -ray.d, ray.time, ray.medium, mp.phase);

                        // Sample new direction at real-scattering event
                        Point2f u = sampler.Get2D();
                        pstd::optional<PhaseFunctionSample> ps = mediumInteraction.phase.Sample_p(-ray.d, u);
                        if (!ps || ps->pdf == 0)
                            terminated = true;
                        else {
                            // Update ray path state for indirect volume scattering
                            scattered = true;
                            ray.o = p;
                            ray.d = ps->wi;
                        }

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
            if (scattered) {
                continue;
            }
        }

        if (!shapeIntersection)
            break;

        shapeIntersection->intr.SkipIntersection(&ray, shapeIntersection->tHit);
    }
}

std::unique_ptr<IntegrationAnalyzer> IntegrationAnalyzer::Create(
    const ParameterDictionary& parameters, Camera camera, Sampler sampler,
    Primitive aggregate, std::vector<Light> lights) {

    int maxDepthFromFile = parameters.GetOneInt("maxdepth", 1); // suppress PBRT warning
    int maxDepth = Options->maxdepth ? Options->maxdepth.value() : maxDepthFromFile;

    return std::make_unique<IntegrationAnalyzer>(maxDepth,
        std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights));
}

}
