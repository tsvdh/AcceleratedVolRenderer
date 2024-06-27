#include "vol_transmittance.h"

namespace graph {

inline void TracePath(Vertex* surfacePoint, FreeGraph* pathGraph) {

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
        if (pair.first == 22)
            auto bla = 42;

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
        TracePath(surfacePoint, pathGraph);
    }
    return pathGraph;
}

}