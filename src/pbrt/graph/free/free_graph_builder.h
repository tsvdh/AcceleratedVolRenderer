#pragma once

#include <pbrt/base/sampler.h>
#include "../graph.h"

namespace graph {
using namespace pbrt;

class FreeGraphBuilder {
public:
    FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, const GraphBuilderConfig& config, bool quiet,
                     int sampleIndexOffset = 0);
    FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, const GraphBuilderConfig& config, bool quiet,
                     int sampleIndexOffset, float squaredSearchRadius);

    FreeGraph TracePaths();

private:
    void TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit, std::optional<int> startingVertex);
    std::vector<std::tuple<int, float>> GetInRadius(const Point3f& pointRef, float squaredRadius, int vertexId = -1);
    std::optional<std::tuple<int, float>> GetClosestInRadius(const Point3f& pointRef, float squaredRadius, int vertexId = -1);
    // void AddToTreeAndFit(Graph& graph, int startId, int endId);
    void UseAndRemovePathInfo(Graph& graph);
    void ReinforceSparseVertices(FreeGraph& graph);
    void OrderIdsAndRebuildTree(Graph& graph);

    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    util::VerticesHolder vHolder;
    std::unique_ptr<DynamicTreeType> searchTree;
    GraphBuilderConfig config;
    bool quiet;
    float squaredSearchRadius;
    int sampleIndexOffset;

    // stats
    util::Averager inNodePathLengthAverager;
};
}
