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
                     int sampleIndexOffset, float squaredSearchRadius, float squaredNeighbourSearchRadius);

    FreeGraph BuildGraph();

private:
    void TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit, std::optional<int> startingVertex);
    std::vector<nanoflann::ResultItem<int, float>> GetInRadius(const Point3f& pointRef, float squaredRadius);
    std::optional<nanoflann::ResultItem<int, float>> GetClosestInRadius(const Point3f& pointRef, float squaredRadius);
    int CountInRadius(const Point3f& pointRef, float squaredRadius);
    void UseAndRemovePathInfo(Graph& graph);
    void ReinforceSparseVertices(FreeGraph& graph);
    void ReinforceSparseVertices(FreeGraph& graph, const std::vector<int>& sparseVertices, const ReinforcementConfig& reinforcementConfig, int cycle,
        ProgressReporter& progress);
    std::vector<nanoflann::ResultItem<int, float>> GetNClosest(const Point3f& pointRef, int nClosest);
    void ComputeSearchRanges(FreeGraph& graph);

    const util::MediumData& mediumData;
    Vector3f inDirection;
    Sampler sampler;
    util::VerticesHolder vHolder;
    std::unique_ptr<DynamicTreeType> searchTree;
    GraphBuilderConfig config;
    bool quiet;
    float squaredSearchRadius;
    float squaredNeighbourSearchRadius;
    int sampleIndexOffset;

    // stats
    util::Averager inNodePathLengthAverager;
};
}
