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
    void PruneAndClean(FreeGraph& graph);
    void ComputeTransmittance(FreeGraph& graph);

private:
    int FreeGraphBuilder::TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit);
    std::vector<std::tuple<int, float>> FreeGraphBuilder::GetInRadius(const Point3f& pointRef, float squaredRadius, int vertexId = -1);
    std::optional<std::tuple<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef, float squaredRadius, int vertexId = -1);
    void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId);
    void UsePathInfo(Graph& graph);
    void AddExtraEdges(Graph& graph);
    void ReinforceSparseAreas(FreeGraph& graph);
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

    // logging
    int64_t scattersInSameSphere = 0;
    int64_t scattersInSameSphereCorrected = 0;
    int64_t totalScatters = 0;
    int64_t edgesAdded = 0;
    int64_t connected = 0;
    int64_t connectedAgain = 0;
};
}
