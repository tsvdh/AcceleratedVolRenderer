#pragma once

#include "pbrt/lights.h"
#include "pbrt/graph/graph.h"
#include "pbrt/graph/lighting_calculator.h"

namespace graph {

class Subdivider {
public:
    Subdivider(Graph& graph, const util::MediumData& mediumData, Vector3f lightDir, Sampler sampler, float baseRadius, SubDividerConfig config);

    void ComputeSubdivisionEffect(SparseVec& initialLight);

private:
    float Subdivide(const Sphere& sphere, Vector3f inDirection, float graphRadius) const;

    Graph& graph;
    const util::MediumData& mediumData;
    MediumInterface sphereInterface;
    Vector3f lightDir;
    Sampler sampler;
    float baseRadius;
    SubDividerConfig config;
};

}