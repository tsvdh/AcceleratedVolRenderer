#pragma once

#include "../graph.h"
#include <pbrt/base/sampler.h>

namespace graph {

using namespace pbrt;

class FreeGraphBuilder {
public:
    FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, GraphBuilderConfig config, bool quiet);
    FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, GraphBuilderConfig config, float radius, bool quiet);

    FreeGraph TracePaths();
    void ComputeTransmittance(FreeGraph& graph);

    float GetSearchRadius() { return searchRadius; }

private:
    int FreeGraphBuilder::TracePath(RayDifferential& ray, FreeGraph& graph, int maxDepth);
    std::optional<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef);
    void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId);
    static void OrderVertexIds(Graph& graph);
    static void ProcessPaths(Graph& graph);

    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    util::VerticesHolder vHolder;
    std::unique_ptr<DynamicTreeType> searchTree;
    GraphBuilderConfig config;
    float searchRadius;
    bool quiet;
};

}