#include "free_graph_builder.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/options.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, DistantLight* light, Sampler sampler, float radiusModifier)
        : mediumData(mediumData), light(light), sampler(std::move(sampler)) {
    lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));
    searchTree = std::make_unique<DynamicTreeType>(3, vHolder);
    searchRadius = GetSameSpotRadius(mediumData) * radiusModifier;
}

int FreeGraphBuilder::TracePath(RayDifferential& ray, FreeGraph& graph, int maxDepth) {
    int numNewVertices = 0;
    Path& path = graph.AddPath();

    while (true) {
        // Initialize _RNG_ for sampling the majorant transmittance
        uint64_t hash0 = Hash(sampler.Get1D());
        uint64_t hash1 = Hash(sampler.Get1D());
        RNG rng(hash0, hash1);

        std::optional<MediumInteraction> optNewInteraction;

        pstd::optional<ShapeIntersection> optIntersection = mediumData.aggregate->Intersect(ray, Infinity);
        if (!optIntersection)
            return numNewVertices;

        float tMax = optIntersection.value().tHit;

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

        if (optResult) {
            Vertex& newVertex = graph.GetVertex(vHolder.GetList()[optResult->first].first)->get();
            graph.AddVertexToPath(newVertex.id, path.id);
        }
        else {
            Vertex& newVertex = graph.AddVertex(newPoint, VertexData{});
            graph.AddVertexToPath(newVertex.id, path.id);

            vHolder.GetList().emplace_back(newVertex.id, newVertex.point);
            ++numNewVertices;
        }

        // terminate if max depth reached
        if (path.Length() == maxDepth)
            return numNewVertices;

        // Sample new direction at real-scattering event
        Point2f u = sampler.Get2D();
        pstd::optional<PhaseFunctionSample> ps = optNewInteraction->phase.Sample_p(-ray.d, u);

        ray.o = newPoint;
        ray.d = ps->wi;
    }
}

FreeGraph FreeGraphBuilder::TracePaths(int numStepsInDimension, int maxDepth) {
    Vector3f xVector;
    Vector3f yVector;
    CoordinateSystem(lightDir, &xVector, &yVector);

    Point3f origin(mediumData.boundsCenter - lightDir * mediumData.maxDistToCenter * 2);
    origin -= Vector3f((xVector + yVector) * mediumData.maxDistToCenter);

    float stepSize = mediumData.maxDistToCenter * 2 / static_cast<float>(numStepsInDimension + 1);
    xVector *= stepSize;
    yVector *= stepSize;

    ProgressReporter workEstimateProgress(Sqr(numStepsInDimension), "Estimating work needed", false);

    int workNeeded = 0;
    for (int x = 1; x <= numStepsInDimension; ++x) {
        for (int y = 1; y <= numStepsInDimension; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential ray(newOrigin, lightDir);

            if (mediumData.aggregate->Intersect(ray, Infinity))
                ++workNeeded;

            workEstimateProgress.Update();
        }
    }
    workEstimateProgress.Done();

    ProgressReporter tracingProgress(workNeeded, "Tracing light paths", false);

    FreeGraph graph;
    int batchSize = 100;
    int startingId = 0;
    int currentBatch = 0;

    for (int x = 1; x <= numStepsInDimension; ++x) {
        for (int y = 1; y <= numStepsInDimension; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential ray(newOrigin, lightDir);

            auto optShapeIntersection = mediumData.aggregate->Intersect(ray, Infinity);
            if (!optShapeIntersection)
                continue;
            optShapeIntersection->intr.SkipIntersection(&ray, optShapeIntersection->tHit);

            sampler.StartPixelSample(Point2i(x, y), 0);
            int newVertices = TracePath(ray, graph, maxDepth);

            currentBatch += newVertices;

            if (currentBatch >= batchSize) {
                AddToTreeAndFit(graph, startingId, startingId + currentBatch);

                startingId += currentBatch;
                currentBatch = 0;
            }

            tracingProgress.Update();
        }
    }
    tracingProgress.Done();

    OrderVertexIds(graph);
    ProcessPaths(graph);

    return graph;
}

std::optional<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef) {
    std::vector<nanoflann::ResultItem<int, float>> resultItems;
    nanoflann::RadiusResultSet resultSet(searchRadius, resultItems);
    float point[3] = { pointRef.x, pointRef.y, pointRef.z };

    searchTree->findNeighbors(resultSet, point);
    resultSet.sort();

    return resultItems.size() <= 1 ? std::nullopt : std::make_optional(resultItems[1]);
}

inline void MovePathReferences(Vertex& toMoveVertex, Vertex& targetVertex, Graph& graph) {
    for (auto& [pathId, pathIndices] : toMoveVertex.paths) {
        targetVertex.AddPathIndices(pathId, pathIndices);

        Path& path = graph.GetPath(pathId)->get();
        for (auto pathIndex : pathIndices)
            path.vertices[pathIndex] = targetVertex.id;
    }

    toMoveVertex.paths.clear();
}

void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId) {
    searchTree->addPoints(startId, endId - 1); // method needs inclusive end range

    for (int curId = startId; curId < endId; ++curId) {
        std::optional<nanoflann::ResultItem<int, float>> resultItem = GetClosestInRadius(graph.GetVertex(curId)->get().point);

        if (resultItem.has_value()) {
            Vertex& vertexToMove = graph.GetVertex(curId)->get();
            Vertex& targetVertex = graph.GetVertex(resultItem->first)->get();

            MovePathReferences(vertexToMove, targetVertex, graph);

            graph.RemoveVertex(curId);
            searchTree->removePoint(curId);
        }
    }
}

void FreeGraphBuilder::OrderVertexIds(Graph& graph) {
    auto GetLargestTakenId = [&](int currentLargestId) {
        for (int i = currentLargestId; i >= 0; --i)
            if (graph.GetVertex(i).has_value())
                return i;

        return -1;
    };

    std::cout << "Rearranging vertex ids... ";

    int currentLargestId = GetLargestTakenId(graph.GetCurVertexId());

    for (int curId = 0; curId < graph.GetCurVertexId(); ++curId) {
        if (currentLargestId <= curId)
            break;

        OptRef<Vertex> optVertex = graph.GetVertex(curId);

        if (!optVertex.has_value()) {
            Vertex& vertexToMove = graph.GetVertex(currentLargestId)->get();
            Vertex& newVertex = graph.AddVertex(curId, vertexToMove.point, VertexData{});

            MovePathReferences(vertexToMove, newVertex, graph);

            graph.RemoveVertex(vertexToMove.id);
            currentLargestId = GetLargestTakenId(currentLargestId);
        }
    }
    std::cout << "done" << std::endl;
}

void FreeGraphBuilder::ProcessPaths(Graph& graph) {
    std::cout << "Adding edges... ";

    for (auto& [_, path] : graph.GetPaths()) {
        if (path.Length() > 0)
            graph.GetVertex(path.vertices[0])->get().data.type = entry;

        for (int i = 0; i < path.Length() - 1; ++i) {
            int fromId = path.vertices[i];
            int toId = path.vertices[i + 1];

            if (fromId == toId)
                continue;

            auto outEdges = graph.GetVertex(fromId)->get().outEdges;
            auto edgeResult = outEdges.find(toId);

            if (edgeResult == outEdges.end())
                graph.AddEdge(path.vertices[i], path.vertices[i + 1], EdgeData{});
        }
    }
    graph.GetPaths().clear();

    std::cout << "done" << std::endl;
}

void FreeGraphBuilder::ComputeTransmittance(FreeGraph& graph, int edgeIterations) {
    ThreadLocal<Sampler> samplers([this]() { return sampler.Clone(); });

    int numEdges = static_cast<int>(graph.GetEdges().size());
    int workNeeded = numEdges * edgeIterations;
    ProgressReporter progress(workNeeded, "Computing edge transmittance", false);

    int resolutionDimensionSize = Options->graph.samplingResolution->x;
    ParallelFor(0, numEdges, [&](int edgeId) {
        Edge& edge = graph.GetEdge(edgeId)->get();

        Point3f fromPoint = graph.GetVertex(edge.from)->get().point;
        Point3f toPoint = graph.GetVertex(edge.to)->get().point;
        MediumInteraction fromInteraction(fromPoint, Vector3f(), 0, mediumData.medium, nullptr);

        Sampler& samplerClone = samplers.Get();

        int yCoor = edgeId / resolutionDimensionSize;
        int xCoor = edgeId - yCoor * resolutionDimensionSize;

        for (int i = 0; i < edgeIterations; ++i) {
            samplerClone.StartPixelSample(Point2i(xCoor, yCoor), i);

            float Tr = util::Transmittance(fromInteraction, toPoint, mediumData.defaultLambda, samplerClone);
            graph.AddEdge(edge.from, edge.to, EdgeData{Tr, -1, 1});
            progress.Update();
        }
    });
    progress.Done();
}

}