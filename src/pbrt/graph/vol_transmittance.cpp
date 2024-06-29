#include "vol_transmittance.h"
#include "pbrt/media.h"

namespace graph {

void VolTransmittance::TracePath(Vertex* surfacePoint, FreeGraph* pathGraph, Vector3f lightDir) {
    RayDifferential ray(surfacePoint->point - lightDir * mediumData->maxDistToCenter, lightDir);
    float depth = 0;
    Path* path = pathGraph->AddPath();
    Vertex* curVertex = nullptr;

    auto AddNewVertex = [&](Point3f p) {
        Vertex* newVertex = pathGraph->AddVertex(p);

        if (curVertex) {
            Edge* edge = pathGraph->AddEdge(curVertex, newVertex, {}, false).value();
            Graph::AddEdgeToPath(edge, path);
        }
        curVertex = newVertex;
    };

    AddNewVertex(ray.o);

    while (true) {
        pstd::optional<ShapeIntersection> si = mediumData->aggregate->Intersect(ray, Infinity);

        if (ray.medium) {

            // Sample the participating medium
            bool scattered = false, terminated = false;
            Float tMax = si ? si->tHit : Infinity;
            // Initialize _RNG_ for sampling the majorant transmittance
            uint64_t hash0 = Hash(sampler.Get1D());
            uint64_t hash1 = Hash(sampler.Get1D());
            RNG rng(hash0, hash1);

            SampleT_maj(
                    (Ray&) ray, tMax, sampler.Get1D(), rng, mediumData->lambda,
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

                        if (mode == 0 || mode == 1)
                            AddNewVertex(p);

                        if (mode == 0) {
                            // Handle absorption along ray path
                            terminated = true;
                            return false;

                        } else if (mode == 1) {
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
                            }

                            return false;

                        } else {
                            // Handle null scattering along ray path
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
    }

    if (path->edges.empty()) {
        pathGraph->RemoveVertex(curVertex->id);
        pathGraph->RemovePath(path->id);
    }
}

FreeGraph* VolTransmittance::CaptureTransmittance(std::vector<Light> lights) {
    if (lights.size() != 1)
        throw std::runtime_error("Expected exactly one light source");

    Light light = lights[0];
    if (!light.Is<DistantLight>())
        throw std::runtime_error("Expected a directional light");

    auto distantLight = light.Cast<DistantLight>();
    Vector3f lightDir = -Normalize(distantLight->GetRenderFromLight()(Vector3f(0, 0, 1)));

    float maxDistToVertex = mediumData->medium.Is<HomogeneousMedium>()
                            ? boundary->GetSpacing() / 10
                            : boundary->GetSpacing() * 2;

    float maxRayLength = mediumData->maxDistToCenter * 2;

    std::vector<Vertex*> surfacePoints;
    ScratchBuffer buffer;
    for (auto pair : boundary->GetVertices()) {
        Vertex* vertex = pair.second;
        RayDifferential ray(vertex->point - lightDir * maxRayLength, lightDir);

        auto shapeInter = mediumData->aggregate->Intersect(ray, Infinity);
        if (!shapeInter)
            continue;

        shapeInter->intr.SkipIntersection(&ray, shapeInter->tHit);

        bool directlyLit = false;

        auto iter = mediumData->medium.SampleRay((Ray&)ray, Infinity, mediumData->lambda, buffer);
        while (true) {
            auto segment = iter.Next();
            if (!segment)
                break;

            if (segment->sigma_maj[0] == 0)
                continue;

            float distToVertex = Length(vertex->point - ray(segment->tMin));
            if (distToVertex <= maxDistToVertex)
                directlyLit = true;

            break;
        }

        if (directlyLit)
            surfacePoints.push_back(pair.second);
    }

    auto pathGraph = new FreeGraph();
    for (Vertex* surfacePoint : surfacePoints) {
        TracePath(surfacePoint, pathGraph, lightDir);
    }
    return pathGraph;
}

}