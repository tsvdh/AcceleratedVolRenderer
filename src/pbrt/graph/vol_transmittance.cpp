#include "vol_transmittance.h"

#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

VolTransmittance::VolTransmittance(const UniformGraph& boundary, const util::MediumData& mediumData,
                                   DistantLight* light, Sampler sampler)
                                       : boundary(boundary), mediumData(mediumData), light(light),
                                         sampler(std::move(sampler)) {
    lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));
}

void VolTransmittance::TraceTransmittancePath(const Vertex& gridPoint, UniformGraph& grid) {
    Point3f startPoint = gridPoint.point - lightDir * mediumData.maxDistToCenter * 2;
    RayDifferential ray(startPoint, lightDir);
    auto shapeIsect = mediumData.aggregate->Intersect(ray, Infinity).value();
    shapeIsect.intr.SkipIntersection(&ray, shapeIsect.tHit);

    int depth = 0;
    int curVertexId = -1;
    MediumInteraction curIntr;

    while (true) {
        // Initialize _RNG_ for sampling the majorant transmittance
        uint64_t hash0 = Hash(sampler.Get1D());
        uint64_t hash1 = Hash(sampler.Get1D());
        RNG rng(hash0, hash1);

        std::optional<MediumInteraction> optNewIntr;

        pstd::optional<ShapeIntersection> optIsect = mediumData.aggregate->Intersect(ray, Infinity);
        if (!optIsect)
            return;

        float tMax = optIsect.value().tHit;

        // Sample new point on ray
        SampleT_maj(
            (Ray&)ray, tMax, sampler.Get1D(), rng, mediumData.lambda,
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

                    optNewIntr = MediumInteraction(p, -ray.d, ray.time, ray.medium, mp.phase);
                    return false;
                }
                else {
                    // Handle null scattering along ray path
                    return true;
                }
            });

        // Handle terminated, scattered, and unscattered medium rays
        // if no new interaction then path is done
        if (!optNewIntr) {
            return;
        }
        MediumInteraction& newIntr = optNewIntr.value();

        auto [coors, fittedPoint] = grid.FitToGraph(optNewIntr->p());
        OptRef<Vertex> optVertex = grid.GetVertex(coors);
        if (!optVertex) {
            if (curVertexId == -1)
                ErrorExit("Path died silently");
            return;
        }

        int newVertexId = optVertex.value().get().id;

        if (newVertexId == curVertexId)
            continue;

        if (curVertexId != -1)
            grid.AddEdge(curVertexId, newVertexId, EdgeData{Transmittance(curIntr, newIntr)});

        // Sample new direction at real-scattering event
        Point2f u = sampler.Get2D();
        pstd::optional<PhaseFunctionSample> ps = newIntr.phase.Sample_p(-ray.d, u);

        ray.o = newIntr.p();
        ray.d = ps->wi;
        curIntr = newIntr;
        curVertexId = newVertexId;
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

void VolTransmittance::CaptureTransmittance(UniformGraph& grid, int multiplier) {
    if (grid.GetSpacing() != boundary.GetSpacing())
        ErrorExit("Spacing of grid and boundary must be equal");

    if (multiplier < 1)
        ErrorExit("Multiplier must be greater or equal than 1");

    int workNeeded = static_cast<int>(grid.GetVerticesConst().size()) * multiplier;
    auto progress = ProgressReporter(workNeeded, "Tracing lit surface", false);

    for (auto& pair : grid.GetVerticesConst()) {
        for (int j = 0; j < multiplier; ++j) {
            sampler.StartPixelSample(Point2i(pair.first, pair.first), j);
            TraceTransmittancePath(pair.second, grid);
            progress.Update();
        }
    }
    progress.Done();
}

}