#include "voxel_transmittance.h"

#include <iostream>

#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

void VoxelTransmittance::TraceTransmittancePath(Point3f startPoint, Vector3f direction, UniformGraph& grid) {
    RayDifferential ray(startPoint, direction);
    auto optShapeIsect = mediumData.aggregate->Intersect(ray, Infinity);
    if (!optShapeIsect)
        return;
    optShapeIsect->intr.SkipIntersection(&ray, optShapeIsect->tHit);

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
            (Ray&)ray, tMax, sampler.Get1D(), rng, mediumData.defaultLambda,
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
                ++numPathsScatteredOutsideGrid;
            return;
        }

        int newVertexId = optVertex.value().get().id;

        if (newVertexId == curVertexId)
            continue;

        if (curVertexId != -1) {
            float Tr = util::Transmittance(curIntr, newIntr, mediumData.defaultLambda, sampler);
            grid.AddEdge(curVertexId, newVertexId, EdgeData{Tr, -1, 1});
        }

        // Sample new direction at real-scattering event
        Point2f u = sampler.Get2D();
        pstd::optional<PhaseFunctionSample> ps = newIntr.phase.Sample_p(-ray.d, u);

        ray.o = newIntr.p();
        ray.d = ps->wi;
        curIntr = newIntr;
        curVertexId = newVertexId;
    }
}

void VoxelTransmittance::CaptureTransmittance(UniformGraph& grid, float sphereStepDegrees, int spheresPerDimension) {
    if (grid.GetSpacing() != boundary.GetSpacing())
        ErrorExit("Spacing of grid and boundary must be equal");

    if (sphereStepDegrees > 90)
        ErrorExit("Sphere step degrees must be less or equal to 90");

    int workNeeded = static_cast<int>(util::GetSpherePoints(Point3f(), 1, sphereStepDegrees).size())
                     * static_cast<int>(std::pow(spheresPerDimension, 3));
    auto progress = ProgressReporter(workNeeded, "Tracing lit surface", false);

    std::array<float, 3> dimensionStepSize{};
    for (int i = 0; i < 3; ++i) {
        dimensionStepSize[i] = mediumData.bounds.Diagonal()[i] / static_cast<float>(spheresPerDimension + 1);
    }

    numPathsScatteredOutsideGrid = 0;

    int curSphere = 0;
    for (int x = 1; x <= spheresPerDimension; ++x) {
        for (int y = 1; y <= spheresPerDimension; ++y) {
            for (int z = 1; z <= spheresPerDimension; ++z) {
                Point3f center = mediumData.bounds.pMin + Vector3f(static_cast<float>(x) * dimensionStepSize[0],
                                                                   static_cast<float>(y) * dimensionStepSize[1],
                                                                   static_cast<float>(z) * dimensionStepSize[2]);

                std::vector<Point3f> spherePoints = util::GetSpherePoints(center, mediumData.maxDistToCenter * 2, sphereStepDegrees);

                for (int i = 0; i < spherePoints.size(); ++i) {
                    sampler.StartPixelSample(Point2i(i, curSphere), 0);

                    Vector3f dir = Normalize(center - spherePoints[i]);
                    TraceTransmittancePath(spherePoints[i], dir, grid);
                    progress.Update();
                }

                ++curSphere;
            }
        }
    }

    progress.Done();
    std::cout << numPathsScatteredOutsideGrid << " / " << workNeeded << " paths died scattering outside grid" << std::endl;
}

}