#pragma once

#include "../graph.h"
#include <pbrt/base/sampler.h>

namespace graph {

using namespace pbrt;

class FreeGraphBuilder {
public:
    explicit FreeGraphBuilder(const util::MediumData& mediumData, DistantLight* light);

    FreeGraph TracePaths(int numSteps, int maxDepth);
    void ConnectVertices(FreeGraph& graph);
    void ComputeTransmittance(FreeGraph& graph, int edgeIterations);

private:
    void FreeGraphBuilder::TracePath(RayDifferential& ray, FreeGraph& graph, int maxDepth);

    const util::MediumData& mediumData;
    DistantLight* light;
    Vector3f lightDir;
    Sampler sampler;
};

}