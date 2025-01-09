#include "free_graph_builder.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/options.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, GraphBuilderConfig config, bool quiet,
                                   bool runInParallel, int sampleIndexOffset)
    : FreeGraphBuilder(mediumData, inDirection, std::move(sampler), std::move(config), quiet, runInParallel, sampleIndexOffset,
                       Sqr(GetSameSpotRadius(mediumData) * config.radiusModifier)) {
}

FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, GraphBuilderConfig config, bool quiet,
                                   bool runInParallel, int sampleIndexOffset, float radius)
    : mediumData(mediumData), inDirection(inDirection), sampler(std::move(sampler)), config(std::move(config)), quiet(quiet), runInParallel(runInParallel),
      searchRadius(radius), sampleIndexOffset(sampleIndexOffset) {
    searchTree = std::make_unique<DynamicTreeType>(3, vHolder);
}

int FreeGraphBuilder::TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit) {
    int numNewVertices = 0;
    std::vector<int> path;
    bool usedTHit = false;

    while (true) {
        // Initialize _RNG_ for sampling the majorant transmittance
        uint64_t hash0 = Hash(sampler.Get1D());
        uint64_t hash1 = Hash(sampler.Get1D());
        RNG rng(hash0, hash1);

        std::optional<MediumInteraction> optNewInteraction;

        float tMax;
        if (!usedTHit) {
            tMax = firstSegmentTHit;
            usedTHit = true;
        }
        else {
            pstd::optional<ShapeIntersection> optIntersection = mediumData.primitiveData.primitive.Intersect(ray, Infinity);
            if (!optIntersection)
                return numNewVertices;

            tMax = optIntersection.value().tHit;
        }

        // Sample new point on ray
        SampleT_maj(
            (Ray&)ray, tMax, sampler.Get1D(), rng, mediumData.defaultLambda,
            [&](Point3f p, MediumProperties mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {
                // Handle medium scattering event for ray

                // Compute medium event probabilities for interaction
                Float pAbsorb = mp.sigma_a[0] / sigma_maj[0];
                Float pScatter = mp.sigma_s[0] / sigma_maj[0];
                Float pNull = std::max<Float>(0, 1 - pAbsorb - pScatter);

                CHECK_GE(1 - pAbsorb - pScatter, -1e-6);
                // Sample medium scattering event type and update path
                Float um = rng.Uniform<Float>();
                int mode = SampleDiscrete({pAbsorb, pScatter, pNull}, um);

                if (mode == 0) {
                    // Handle absorption along ray path
                    return false;
                }
                if (mode == 1) {
                    // Handle scattering along ray path

                    optNewInteraction = MediumInteraction(p, -ray.d, ray.time, ray.medium, mp.phase);
                    return false;
                }
                else {
                    // Handle null scattering along ray path
                    return true;
                }
            });

        // Handle terminated, scattered, and unscattered medium rays
        // if no new interaction then path is done
        if (!optNewInteraction) {
            return numNewVertices;
        }

        Point3f newPoint = optNewInteraction->p();
        // ReSharper disable once CppTooWideScope
        std::optional<nanoflann::ResultItem<int, float>> optResult = GetClosestInRadius(newPoint);

        const Vertex* newVertex;
        if (optResult) {
            newVertex = &graph.GetVertex(vHolder.GetList()[optResult->first].first)->get();
        } else {
            newVertex = &graph.AddVertex(newPoint, VertexData{});

            vHolder.GetList().emplace_back(newVertex->id, newVertex->point);
            ++numNewVertices;
        }

        path.push_back(newVertex->id);
        int pathSize = static_cast<int>(path.size());

        if (pathSize == 1)
            graph.GetVertex(path[0])->get().data.type = entry;
        else { // path > 1
            int from = path[pathSize - 2];
            int to = path[pathSize - 1];

            Vertex& fromVertex = graph.GetVertex(from)->get();
            if (from != to && fromVertex.outEdges.find(to) == fromVertex.outEdges.end())
                graph.AddEdge(from, to, EdgeData{});
        }

        // terminate if max depth reached
        if (pathSize == maxDepth)
            return numNewVertices;

        // Sample new direction at real-scattering event
        Point2f u = sampler.Get2D();
        pstd::optional<PhaseFunctionSample> ps = optNewInteraction->phase.Sample_p(-ray.d, u);

        ray.o = newPoint;
        ray.d = ps->wi;
    }
}

FreeGraph FreeGraphBuilder::TracePaths() {
    Vector3f xVector;
    Vector3f yVector;
    CoordinateSystem(inDirection, &xVector, &yVector);

    const util::PrimitiveData& primitiveData = mediumData.primitiveData;
    Point3f origin(primitiveData.boundsCenter - inDirection * primitiveData.maxDistToCenter * 2);
    origin -= Vector3f((xVector + yVector) * primitiveData.maxDistToCenter);

    float stepSize = primitiveData.maxDistToCenter * 2 / static_cast<float>(config.dimensionSteps + 1);
    xVector *= stepSize;
    yVector *= stepSize;

    ProgressReporter workEstimateProgress(Sqr(config.dimensionSteps), "Estimating work needed", quiet);

    int workNeeded = 0;
    for (int x = 1; x <= config.dimensionSteps; ++x) {
        for (int y = 1; y <= config.dimensionSteps; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential ray(newOrigin, inDirection);

            if (primitiveData.primitive.Intersect(ray, Infinity))
                ++workNeeded;

            workEstimateProgress.Update();
        }
    }
    workNeeded *= config.iterationsPerStep;
    workEstimateProgress.Done();

    ProgressReporter tracingProgress(workNeeded, "Tracing light paths", quiet);

    FreeGraph graph;
    int batchSize = 100;
    int startingId = 0;
    int currentBatch = 0;

    int resolutionDimensionSize = Options->graph.samplingResolution->x;

    for (int x = 1; x <= config.dimensionSteps; ++x) {
        for (int y = 1; y <= config.dimensionSteps; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential ray(newOrigin, inDirection, 0, mediumData.medium);

            auto optShapeIntersection = primitiveData.primitive.Intersect(ray, Infinity);
            if (!optShapeIntersection)
                continue;
            optShapeIntersection->intr.SkipIntersection(&ray, optShapeIntersection->tHit);

            pstd::optional<ShapeIntersection> optIntersection = mediumData.primitiveData.primitive.Intersect(ray, Infinity);
            if (!optIntersection)
                continue;
            float firstSegmentTHit = optIntersection.value().tHit;

            int startIndex = ((x - 1) + (y - 1) * config.dimensionSteps) * config.iterationsPerStep;
            for (int i = 0; i < config.iterationsPerStep; ++i) {
                int curIndex = startIndex + i;

                int yCoor = curIndex / resolutionDimensionSize;
                int xCoor = curIndex - yCoor * resolutionDimensionSize;

                sampler.StartPixelSample(Point2i(xCoor, yCoor), sampleIndexOffset);
                int newVertices = TracePath(ray, graph, config.maxDepth, firstSegmentTHit);

                currentBatch += newVertices;

                if (currentBatch >= batchSize) {
                    AddToTreeAndFit(graph, startingId, startingId + currentBatch);

                    startingId += currentBatch;
                    currentBatch = 0;
                }

                tracingProgress.Update();
            }
        }
    }
    AddToTreeAndFit(graph, startingId, startingId + currentBatch);
    tracingProgress.Done();

    OrderVertexIds(graph);
    return graph;
}

std::optional<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef) {
    std::vector<nanoflann::ResultItem<int, float>> resultItems;
    nanoflann::RadiusResultSet resultSet(searchRadius, resultItems);
    float point[3] = {pointRef.x, pointRef.y, pointRef.z};

    searchTree->findNeighbors(resultSet, point);
    resultSet.sort();

    return resultItems.size() <= 1 ? std::nullopt : std::make_optional(resultItems[1]);
}

inline void MoveEdgeReferences(Vertex& toMoveVertex, Vertex& targetVertex, Graph& graph) {
    struct TempEdge {
        TempEdge(int id, int from, int to) : id(id), from(from), to(to) {}
        int id, from, to;
    };

    std::vector<TempEdge> edgesToAdd;
    std::vector<int> edgesToRemove;

    for (auto& [fromId, edgeId] : toMoveVertex.inEdges) {
        Vertex& fromVertex = graph.GetVertex(fromId)->get();

        if (fromVertex.id == targetVertex.id)
            edgesToRemove.push_back(edgeId);
        else if (targetVertex.inEdges.find(fromVertex.id) == targetVertex.inEdges.end()) {
            edgesToAdd.emplace_back(edgeId, fromVertex.id, targetVertex.id);
            edgesToRemove.push_back(edgeId);
        }
        else
            edgesToRemove.push_back(edgeId);
    }

    for (auto& [toId, edgeId] : toMoveVertex.outEdges) {
        Vertex& toVertex = graph.GetVertex(toId)->get();

        if (toVertex.id == targetVertex.id)
            edgesToRemove.push_back(edgeId);
        else if (targetVertex.outEdges.find(toVertex.id) == targetVertex.outEdges.end()) {
            edgesToAdd.emplace_back(edgeId, targetVertex.id, toVertex.id);
            edgesToRemove.push_back(edgeId);
        }
        else
            edgesToRemove.push_back(edgeId);
    }

    for (int toRemoveId : edgesToRemove)
        graph.RemoveEdge(toRemoveId);

    for (TempEdge toAdd : edgesToAdd)
        graph.AddEdge(toAdd.id, toAdd.from, toAdd.to, EdgeData{});
}

inline void MoveVertexToTarget(Vertex& vertexToMove, Vertex& targetVertex, Graph& graph) {
    if (vertexToMove.data.type == entry)
        targetVertex.data.type = entry;

    MoveEdgeReferences(vertexToMove, targetVertex, graph);

    graph.RemoveVertex(vertexToMove.id);
}

void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId) {
    searchTree->addPoints(startId, endId - 1); // method needs inclusive end range

    for (int curId = startId; curId < endId; ++curId) {
        std::optional<nanoflann::ResultItem<int, float>> resultItem = GetClosestInRadius(graph.GetVertex(curId)->get().point);

        if (resultItem.has_value()) {
            Vertex& vertexToMove = graph.GetVertex(curId)->get();
            Vertex& targetVertex = graph.GetVertex(resultItem->first)->get();

            MoveVertexToTarget(vertexToMove, targetVertex, graph);

            searchTree->removePoint(curId);
        }
    }
}

void FreeGraphBuilder::OrderVertexIds(Graph& graph) const {
    auto GetLargestTakenId = [&](int currentLargestId) {
        for (int i = currentLargestId; i >= 0; --i)
            if (graph.GetVertex(i).has_value())
                return i;

        return -1;
    };

    auto startTime = std::chrono::high_resolution_clock::now();
    if (!quiet)
        std::cout << "Rearranging vertex ids... ";

    int currentLargestId = GetLargestTakenId(graph.GetCurVertexId());

    for (int curId = 0; curId < graph.GetCurVertexId(); ++curId) {
        if (currentLargestId <= curId)
            break;

        OptRef<Vertex> optVertex = graph.GetVertex(curId);

        if (!optVertex.has_value()) {
            Vertex& vertexToMove = graph.GetVertex(currentLargestId)->get();
            Vertex& newVertex = graph.AddVertex(curId, vertexToMove.point, VertexData{});

            MoveVertexToTarget(vertexToMove, newVertex, graph);

            currentLargestId = GetLargestTakenId(currentLargestId);
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    if (!quiet)
        std::cout << "done (" << duration << "s)" << std::endl;
}

void FreeGraphBuilder::ComputeTransmittance(FreeGraph& graph) {
    ThreadLocal<Sampler> samplers([this]() { return sampler.Clone(); });

    int numEdges = static_cast<int>(graph.GetEdges().size());
    int workNeeded = numEdges * config.edgeIterations;
    if (workNeeded == 0)
        return;

    std::vector<int> edgeIds;
    edgeIds.reserve(numEdges);
    for (auto& [id, _] : graph.GetEdges())
        edgeIds.push_back(id);

    int resolutionDimensionSize = Options->graph.samplingResolution->x;

    ProgressReporter progress(workNeeded, "Computing edge transmittance", quiet);

    ParallelFor(0, numEdges, runInParallel, [&](int listIndex) {
        int edgeId = edgeIds[listIndex];
        Edge& edge = graph.GetEdge(edgeId)->get();

        Point3f fromPoint = graph.GetVertex(edge.from)->get().point;
        Point3f toPoint = graph.GetVertex(edge.to)->get().point;
        MediumInteraction fromInteraction(fromPoint, Vector3f(), 0, mediumData.medium, nullptr);

        Sampler& samplerClone = samplers.Get();

        int yCoor = listIndex / resolutionDimensionSize;
        int xCoor = listIndex - yCoor * resolutionDimensionSize;

        for (int i = 0; i < config.edgeIterations; ++i) {
            samplerClone.StartPixelSample(Point2i(xCoor, yCoor), i);

            float Tr = util::Transmittance(fromInteraction, toPoint, mediumData.defaultLambda, samplerClone);
            graph.AddEdge(edge.from, edge.to, EdgeData{Tr, -1, 1});
            progress.Update();
        }
    });
    progress.Done();
}
}
