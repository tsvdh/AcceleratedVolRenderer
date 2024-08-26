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

void VolTransmittance::TracePath(const Vertex& surfacePoint, FreeGraph& pathGraph, Vector3f lightDir) {
    RayDifferential ray(surfacePoint.point - lightDir * mediumData.maxDistToCenter, lightDir);
    float depth = 0;
    Path& path = pathGraph.AddPath();
    Vertex curVertex;
    SampledSpectrum curPhase(1);
    float curPhaseRatio = 1;
    bool insideMedium = false;

    auto AddNewPathSegment = [&](Point3f p, VertexData vertexData, EdgeData edgeData) {
        Vertex& newVertex = pathGraph.AddVertex(p, vertexData);

        if (curVertex.id != -1) {
            Edge& edge = pathGraph.AddEdge(curVertex.id, newVertex.id, edgeData).value();
            pathGraph.AddEdgeToPath(edge.id, path.id);
        }
        curVertex = newVertex;
    };

    AddNewPathSegment(ray.o, VertexData{}, EdgeData{});

    while (true) {
        pstd::optional<ShapeIntersection> si = mediumData.aggregate->Intersect(ray, Infinity);

        if (ray.medium) {

            // Sample the participating medium
            bool scattered = false, terminated = false;
            Float tMax = si ? si->tHit : Infinity;
            // Initialize _RNG_ for sampling the majorant transmittance
            uint64_t hash0 = Hash(sampler.Get1D());
            uint64_t hash1 = Hash(sampler.Get1D());
            RNG rng(hash0, hash1);

            SampledSpectrum segT_Maj(1);

            SampleT_maj(
                (Ray&) ray, tMax, sampler.Get1D(), rng, mediumData.lambda,
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

                    segT_Maj *= T_maj;
                    if (mode == 0 || mode == 1) {
                        int visibility = 1; // TODO: check visibility
                        float dist = std::max(1.f, Length(curVertex.point - p));
                        float G = static_cast<float>(visibility) / Sqr(dist);
                        SampledSpectrum throughput = curPhase * segT_Maj * G;

                        VertexData vertexData{mode == 0 ? absorp : scatter};
                        EdgeData edgeData{throughput, curPhaseRatio};
                        AddNewPathSegment(p, vertexData, edgeData);
                    }
                    if (mode == 0) {
                        // Handle absorption along ray path
                        terminated = true;
                        return false;
                    }
                    else if (mode == 1) {
                        // Handle scattering along ray path
                        // Stop path sampling if maximum depth has been reached
                        if (depth++ >= maxDepth) {
                            terminated = true;
                            return false;
                        }

                        MediumInteraction intr(p, -ray.d, ray.time, ray.medium, mp.phase);

                        // Sample new direction at real-scattering event
                        Point2f u = sampler.Get2D();
                        pstd::optional<PhaseFunctionSample> ps = intr.phase.Sample_p(-ray.d, u);
                        if (!ps || ps->pdf == 0)
                            terminated = true;
                        else {
                            // Update ray path state for indirect volume scattering
                            scattered = true;
                            ray.o = p;
                            ray.d = ps->wi;

                            curPhase = mp.sigma_s * ps->p;
                            curPhaseRatio = ps->p / ps->pdf;
                        }

                        return false;
                    }
                    else {
                        // Handle null scattering along ray path
                        curPhase *= sigma_maj - mp.sigma_a - mp.sigma_s;
                        return true;
                    }
                });

            // Handle terminated, scattered, and unscattered medium rays
            if (terminated)
                break;
            if (scattered)
                continue;
        }

        // Handle surviving unscattered rays
        if (!si)
            break;

        si->intr.SkipIntersection(&ray, si->tHit);
        if (!insideMedium) {
            AddNewPathSegment(ray.o,
                VertexData{entry},
                EdgeData{SampledSpectrum(1), 1});
            insideMedium = true;
        }
    }

    if (path.edges.empty()) {
        pathGraph.RemoveVertex(curVertex.id);
        pathGraph.RemovePath(path.id);
    }
}

FreeGraph VolTransmittance::CaptureTransmittance(const std::vector<Light>& lights, float amount) {
    if (amount < 0 || amount > 1)
        ErrorExit("Amount should be a ratio");

    if (lights.size() != 1)
        ErrorExit("Expected exactly one light source");

    Light light = lights[0];
    if (!light.Is<DistantLight>())
        ErrorExit("Expected a directional light");

    auto distantLight = light.Cast<DistantLight>();
    Vector3f lightDir = -Normalize(distantLight->GetRenderFromLight()(Vector3f(0, 0, 1)));

    std::vector<Ref<const Vertex>> litSurfacePoints = GetLitSurfacePoints(lightDir);

    std::vector<Ref<const Vertex>> selectedSurfacePoints;
    int increment = static_cast<int>(std::round(1 / amount));
    for (int i = 0; i < litSurfacePoints.size(); i += increment) {
        selectedSurfacePoints.push_back(litSurfacePoints[i]);
    }

    auto progress = ProgressReporter(static_cast<int>(selectedSurfacePoints.size()), "Tracing lit surface", false);

    FreeGraph pathGraph;
    for (int i = 0; i < selectedSurfacePoints.size(); ++i) {
        sampler.StartPixelSample(Point2i(i, 0), 0);
        TracePath(selectedSurfacePoints[i], pathGraph, lightDir);
        progress.Update();
    }
    progress.Done();

    // for (auto& pair : pathGraph.GetPaths()) {
    //     const Path& path = pair.second;
    //
    //     std::vector<Ref<Edge>> edges = path.edges;
    //
    //     for (int i = 0; i < edges.size(); ++i) {
    //         EdgeData* data = edges[i].data;
    //
    //         SampledSpectrum prevThroughput = i == 0 ? SampledSpectrum(1) : path->edgeData[i - 1]->throughput;
    //         float prevWeightedThroughput = i == 0 ? 1 : path->edgeData[i - 1]->weightedThroughput;
    //         path->edgeData[i] = new EdgeData{prevThroughput * data->throughput,
    //                                          prevWeightedThroughput * data->weightedThroughput};
    //     }
    //
    //     VertexData* finalData = edges.back()->to->data;
    //     if (finalData->type == scatter)
    //         finalData->type = scatter_final;
    // }

    return pathGraph;
}

}