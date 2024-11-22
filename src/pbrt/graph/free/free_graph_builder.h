#pragma once

#include "../graph.h"
#include <pbrt/base/sampler.h>

namespace graph {

using namespace pbrt;

class FreeGraphBuilder {
public:
    FreeGraphBuilder(const util::MediumData& mediumData, DistantLight* light, Sampler sampler);

    FreeGraph TracePaths(int numStepsInDimension, int maxDepth);
    void ComputeTransmittance(FreeGraph& graph, int edgeIterations);

private:
    int FreeGraphBuilder::TracePath(RayDifferential& ray, FreeGraph& graph, int maxDepth);
    std::optional<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetClosestInRadius(Graph& graph, int vertexId);
    void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId);
    static void OrderVertexIds(Graph& graph);
    static void ProcessPaths(Graph& graph);

    const util::MediumData& mediumData;
    DistantLight* light;
    Vector3f lightDir;
    Sampler sampler;
    util::VerticesHolder vHolder;
    std::unique_ptr<DynamicTreeType> searchTree;
    float searchRadius;
};

}