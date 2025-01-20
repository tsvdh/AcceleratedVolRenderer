#pragma once

#include "pbrt/lights.h"
#include "pbrt/graph/graph.h"
#include "pbrt/graph/lighting_calculator.h"

namespace graph {
class Subdivider {
public:
    Subdivider(FreeGraph& graph, const util::MediumData& mediumData, Vector3f lightDir, Sampler sampler, const SubDividerConfig& config);

    void ComputeSubdivisionEffect(SparseVec& initialLight);

private:
    float Subdivide(int vertexId, const Sphere& sphere, Vector3f inDirection, float squaredSearchRadius);

    FreeGraph& graph;
    const util::MediumData& mediumData;
    MediumInterface sphereInterface;
    Vector3f lightDir;
    Sampler sampler;
    SubDividerConfig config;
};
}
