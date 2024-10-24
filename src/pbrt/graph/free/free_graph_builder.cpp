#include "free_graph_builder.h"

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, DistantLight* light, Sampler sampler)
        : mediumData(mediumData), light(light), sampler(std::move(sampler)) {
    lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));
}

void FreeGraphBuilder::TracePath(RayDifferential& ray, FreeGraph& graph, int maxDepth) {
    auto optShapeIsect = mediumData.aggregate->Intersect(ray, Infinity);
    if (!optShapeIsect)
        return;
    optShapeIsect->intr.SkipIntersection(&ray, optShapeIsect->tHit);

    int curId = -1;
    int depth = 0;

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

        Vertex& newVertex = graph.AddVertex(optNewIntr->p(), VertexData{});

        if (curId == -1) {
            newVertex.data.type = entry;
        }
        else {
            graph.AddEdge(curId, newVertex.id, EdgeData{});
        }

        // Sample new direction at real-scattering event
        Point2f u = sampler.Get2D();
        pstd::optional<PhaseFunctionSample> ps = optNewIntr->phase.Sample_p(-ray.d, u);

        ray.o = newVertex.point;
        ray.d = ps->wi;
        curId = newVertex.id;

        // Stop path sampling if maximum depth has been reached
        if (++depth == maxDepth)
            return;
    }
}

FreeGraph FreeGraphBuilder::TracePaths(int numStepsInDimension, int maxDepth) {
    Vector3f xVector;
    Vector3f yVector;
    CoordinateSystem(lightDir, &xVector, &yVector);

    Point3f origin(mediumData.boundsCenter - lightDir * mediumData.maxDistToCenter * 2);
    origin -= Vector3f((xVector + yVector) * mediumData.maxDistToCenter);

    float stepSize = mediumData.maxDistToCenter * 2 / static_cast<float>(numStepsInDimension + 1);
    xVector *= stepSize;
    yVector *= stepSize;

    FreeGraph graph;

    int workNeeded = static_cast<int>(std::pow(numStepsInDimension, 2));
    ProgressReporter progress(workNeeded, "Tracing light paths", false);

    for (int x = 1; x <= numStepsInDimension; ++x) {
        for (int y = 1; y <= numStepsInDimension; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential ray(newOrigin, lightDir);

            sampler.StartPixelSample(Point2i(x, y), 0);
            TracePath(ray, graph, maxDepth);

            progress.Update();
        }
    }
    progress.Done();

    return graph;
}

void FreeGraphBuilder::ConnectVertices(FreeGraph& graph) {}

void FreeGraphBuilder::ComputeTransmittance(FreeGraph& graph, int edgeIterations) {
    int workNeeded = static_cast<int>(graph.GetEdges().size() * edgeIterations);
    ProgressReporter progress(workNeeded, "Computing edge transmittance", false);

    for (auto& pair : graph.GetEdges()) {
        int fromId = pair.second.from;
        int toId = pair.second.to;
        Point3f fromPoint = graph.GetVertex(fromId)->get().point;
        Point3f toPoint = graph.GetVertex(toId)->get().point;
        MediumInteraction p0(fromPoint, Vector3f(), 0, mediumData.medium, nullptr);
        MediumInteraction p1(toPoint, Vector3f(), 0, mediumData.medium, nullptr);

        for (int i = 0; i < edgeIterations; ++i) {
            float Tr = Transmittance(p0, p1, mediumData.defaultLambda);
            graph.AddEdge(fromId, toId, EdgeData{Tr, -1, 1});
            progress.Update();
        }
    }
    progress.Done();
}

}