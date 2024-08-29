#include "vol_transmittance.h"

#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

std::vector<Ref<const Vertex>> VolTransmittance::GetLitSurfacePoints(Vector3f lightDir) {
    float maxDistToVertex = mediumData.medium.Is<HomogeneousMedium>()
                            ? boundary.GetSpacing()
                            : boundary.GetSpacing() * 4;

    float maxRayLength = mediumData.maxDistToCenter * 2;

    std::vector<Ref<const Vertex>> litSurfacePoints;

    ScratchBuffer buffer;
    for (auto& pair : boundary.GetVertices()) {
        const Vertex& vertex = pair.second;

        RayDifferential ray(vertex.point - lightDir * maxRayLength, lightDir);

        auto shapeInter = mediumData.aggregate->Intersect(ray, Infinity);
        if (!shapeInter)
            continue;

        shapeInter->intr.SkipIntersection(&ray, shapeInter->tHit);

        bool directlyLit = false;

        auto iter = mediumData.medium.SampleRay((Ray&)ray, Infinity, mediumData.lambda, buffer);
        while (true) {
            auto segment = iter.Next();
            if (!segment)
                break;

            if (segment->sigma_maj[0] == 0)
                continue;

            float distToVertex = Length(vertex.point - ray(segment->tMin));
            if (distToVertex <= maxDistToVertex)
                directlyLit = true;

            break;
        }

        if (directlyLit)
            litSurfacePoints.push_back(std::ref(vertex));
    }

    return litSurfacePoints;
}

void VolTransmittance::TracePath(const Vertex& surfacePoint, UniformGraph& grid, Vector3f lightDir) {
    Vertex curVertex = surfacePoint;
    RayDifferential ray(curVertex.point, lightDir);
    int depth = 0;
    bool scattered = false;

    MediumInteraction curIntr(curVertex.point, -lightDir, 0, mediumData.medium,
        mediumData.medium.SamplePoint(curVertex.point, mediumData.lambda).phase);

    while (true) {
        // Initialize _RNG_ for sampling the majorant transmittance
        uint64_t hash0 = Hash(sampler.Get1D());
        uint64_t hash1 = Hash(sampler.Get1D());
        RNG rng(hash0, hash1);

        MediumInteraction newIntr;

        // Sample new point on ray
        SampleT_maj(
            (Ray&)ray, Infinity, sampler.Get1D(), rng, mediumData.lambda,
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
                    // Stop path sampling if maximum depth has been reached
                    if (depth++ >= maxDepth) {
                        return false;
                    }

                    newIntr = MediumInteraction(p, -ray.d, ray.time, ray.medium, mp.phase);

                    scattered = true;
                    return false;
                }
                else {
                    // Handle null scattering along ray path
                    return true;
                }
            });

        // Handle terminated, scattered, and unscattered medium rays
        // if unscattered then path is done
        if (!scattered) {
            return;
        }

        auto [coors, fittedPoint] = grid.FitToGraph(newIntr.p());
        OptRef<Vertex> optVertex = grid.GetVertex(coors);
        if (!optVertex) {
            return;
        }

        Vertex& newVertex = optVertex.value().get();

        if (newVertex == curVertex)
            continue;

        grid.AddEdge(curVertex.id, newVertex.id, EdgeData{Transmittance(curIntr, newIntr)});

        // Sample new direction at real-scattering event
        Point2f u = sampler.Get2D();
        pstd::optional<PhaseFunctionSample> ps = newIntr.phase.Sample_p(-ray.d, u);

        ray.o = newIntr.p();
        ray.d = ps->wi;
        curIntr = newIntr;
        curVertex = newVertex;
    }
}

SampledSpectrum VolTransmittance::Transmittance(const MediumInteraction& p0, const MediumInteraction& p1) const {
    RNG rng(Hash(p0.p()), Hash(p1.p()));

    Ray ray = p0.SpawnRayTo(p1);
    SampledSpectrum Tr(1.f), inv_w(1.f);
    if (LengthSquared(ray.d) == 0)
        return Tr;

    SampledSpectrum T_majRemain = SampleT_maj(ray, 1.f, rng.Uniform<Float>(), rng, mediumData.lambda,
        [&](Point3f p, const MediumProperties& mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {

            SampledSpectrum sigma_n = ClampZero(sigma_maj - mp.sigma_a - mp.sigma_s);

            // ratio-tracking: only evaluate null scattering
            Float pdf = T_maj[0] * sigma_maj[0];
            Tr *= T_maj * sigma_n / pdf;
            inv_w *= T_maj * sigma_maj / pdf;

            if (!Tr || !inv_w)
                return false;

            return true;
        });

    Tr *= T_majRemain / T_majRemain[0];
    inv_w *= T_majRemain / T_majRemain[0];


    return Tr / inv_w.Average();
}

void VolTransmittance::CaptureTransmittance(UniformGraph& grid, const std::vector<Light>& lights, float amount, int multiplier) {
    if (grid.GetSpacing() != boundary.GetSpacing())
        ErrorExit("Spacing of grid and boundary must be equal");

    if (lights.size() != 1)
        ErrorExit("Expected exactly one light source");

    Light light = lights[0];
    if (!light.Is<DistantLight>())
        ErrorExit("Expected a directional light");

    if (amount < 0 || amount > 1)
        ErrorExit("Amount should be a ratio");

    if (multiplier < 1)
        ErrorExit("Multiplier must be greater or equal than 1");

    auto distantLight = light.Cast<DistantLight>();
    Vector3f lightDir = -Normalize(distantLight->GetRenderFromLight()(Vector3f(0, 0, 1)));

    std::vector<Ref<const Vertex>> litSurfacePoints = GetLitSurfacePoints(lightDir);

    std::vector<Ref<const Vertex>> selectedSurfacePoints;
    int increment = static_cast<int>(std::round(1 / amount));
    for (int i = 0; i < litSurfacePoints.size(); i += increment) {
        selectedSurfacePoints.push_back(litSurfacePoints[i]);
    }

    int workNeeded = static_cast<int>(selectedSurfacePoints.size()) * multiplier;
    auto progress = ProgressReporter(workNeeded, "Tracing lit surface", false);

    for (int i = 0; i < selectedSurfacePoints.size(); ++i) {
        for (int j = 0; j < multiplier; ++j) {
            sampler.StartPixelSample(Point2i(i, j), 0);
            TracePath(boundary.GetVertex(1).value(), grid, lightDir);
            progress.Update();
        }
    }
    progress.Done();
}

}