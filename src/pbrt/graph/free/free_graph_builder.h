#pragma once

#include <pbrt/base/sampler.h>
#include "../graph.h"

namespace graph {
using namespace pbrt;

class FreeGraphBuilder {
public:
    FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, GraphBuilderConfig config, bool quiet,
                     bool runInParallel);
    FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, GraphBuilderConfig config, bool quiet,
                     bool runInParallel, float radius);

    FreeGraph TracePaths();
    void ComputeTransmittance(FreeGraph& graph);

    float GetSearchRadius() const { return searchRadius; }

private:
    int FreeGraphBuilder::TracePath(RayDifferential& ray, FreeGraph& graph, int maxDepth);
    std::optional<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef);
    void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId);
    void OrderVertexIds(Graph& graph) const;
    void ProcessPaths(Graph& graph) const;

    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    util::VerticesHolder vHolder;
    std::unique_ptr<DynamicTreeType> searchTree;
    GraphBuilderConfig config;
    bool quiet;
    bool runInParallel;
    float searchRadius;
};
}
