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
    void PruneAndClean(FreeGraph& graph) const;
    void ComputeTransmittance(FreeGraph& graph);

    util::VerticesHolder& temp() {return vHolder;}

private:
    int FreeGraphBuilder::TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit);
    std::optional<std::tuple<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef, int vertexId = -1);
    void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId);
    void OrderVertexIds(Graph& graph) const;
    void UsePathInfo(Graph& graph);
    void AddExtraEdges(Graph& graph);

    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    util::VerticesHolder vHolder;
    std::unique_ptr<DynamicTreeType> searchTree;
    GraphBuilderConfig config;
    bool quiet;
    float squaredSearchRadius;
    int sampleIndexOffset;

    int64_t scattersInSameSphere = 0;
    int64_t scattersInSameSphereCorrected = 0;
    int64_t totalScatters = 0;
    int64_t edgesAdded = 0;
};
}
