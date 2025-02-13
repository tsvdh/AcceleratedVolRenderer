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
    void ComputeTransmittance(FreeGraph& graph);

private:
    int FreeGraphBuilder::TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit);
    std::optional<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef);
    void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId);
    void OrderVertexIds(Graph& graph) const;

    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    util::VerticesHolder vHolder;
    std::unique_ptr<DynamicTreeType> searchTree;
    GraphBuilderConfig config;
    bool quiet;
    float squaredSearchRadius;
    int sampleIndexOffset;

    std::vector<int> pathLengths;
    int scattersInSameSphere = 0;
    int totalScatters = 0;
};
}
