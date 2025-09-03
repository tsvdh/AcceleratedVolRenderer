#include "free_graph_builder.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/options.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, const GraphBuilderConfig& config,
                                   bool quiet, int sampleIndexOffset)
    : FreeGraphBuilder(mediumData, inDirection, std::move(sampler), config, quiet, sampleIndexOffset,
                       Sqr(GetSameSpotRadius(mediumData) * config.radiusModifier)) {
}

FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, const GraphBuilderConfig& config,
                                   bool quiet, int sampleIndexOffset, float squaredSearchRadius)
    : mediumData(mediumData), inDirection(inDirection), sampler(std::move(sampler)), config(config), quiet(quiet),
      squaredSearchRadius(squaredSearchRadius), sampleIndexOffset(sampleIndexOffset) {
    searchTree = std::make_unique<DynamicTreeType>(3, vHolder);
}

int FreeGraphBuilder::TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit) {
    int numNewVertices = 0;
    Path& pathHolder = graph.AddPath();
    std::vector<int>& path = pathHolder.vertices;
    bool usedTHit = false;

    auto HandlePotentialPathEnd = [&] {
        if (!path.empty()) {
            Vertex& lastVertex = graph.GetVertex(path.back())->get();
            ++lastVertex.data.totalSamples;
        }
    };

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

            if (!optIntersection) {
                HandlePotentialPathEnd();
                // pathLengths.push_back(path.size());
                return numNewVertices;
            }

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
        HandlePotentialPathEnd();
        if (!optNewInteraction) {
            // pathLengths.push_back(path.size());
            return numNewVertices;
        }

        Point3f newPoint = optNewInteraction->p();
        std::optional<std::tuple<int, float>> optResult = GetClosestInRadius(newPoint, squaredSearchRadius);
        std::optional<Point3f> prevPoint = path.empty() ? std::nullopt : std::make_optional(graph.GetVertex(path.back())->get().point);

        Vertex* newVertex;
        if (optResult) {
            newVertex = &graph.GetVertex(std::get<0>(optResult.value()))->get();
        }
        else if (prevPoint && DistanceSquared(prevPoint.value(), newPoint) <= squaredSearchRadius) {
            newVertex = &graph.GetVertex(path.back())->get();
        }
        else {
            newVertex = &graph.AddVertex(newPoint, VertexData{});
            vHolder.GetList().emplace_back(newVertex->point);
            ++numNewVertices;
        }

        graph.AddVertexToPath(newVertex->id, pathHolder.id);

        if (path.size() >= 2) {
            int pathSize = pathHolder.Length();

            int from = path[pathSize - 2];
            int to = path[pathSize - 1];

            if (from != to) {
                graph.AddEdge(from, to, EdgeData{1});
            } else
                ++scattersInSameSphere;
        }
        ++totalScatters;

        // terminate if max depth reached
        if (path.size() == maxDepth) {
            // pathLengths.push_back(path.size());
            return numNewVertices;
        }

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

    int64_t workNeeded = 0;
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

    FreeGraph graph(std::sqrt(squaredSearchRadius));

    int batchSize = 100;
    int startingId = graph.GetCurVertexId();
    int currentBatch = 0;

    int resolutionDimensionSize = Options->graph.samplingResolution->x;

    for (int x = 1; x <= config.dimensionSteps; ++x) {
        for (int y = 1; y <= config.dimensionSteps; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential ray(newOrigin, inDirection, 0, mediumData.medium);

            util::HitsResult mediumHits = GetHits(mediumData.primitiveData.primitive, ray, mediumData);

            if (!mediumHits.RayEntersVolume())
                continue;

            if (mediumHits.type == util::OutsideTwoHits)
                mediumHits.intersections[0].intr.SkipIntersection(&ray, mediumHits.tHits[0]);

            float mediumExitTHit = mediumHits.type == util::OutsideTwoHits ? mediumHits.tHits[1] - mediumHits.tHits[0] : mediumHits.tHits[0];

            uint64_t startIndex = ((y - 1) + (x - 1) * config.dimensionSteps) * config.iterationsPerStep;
            for (int i = 0; i < config.iterationsPerStep; ++i) {
                uint64_t curIndex = startIndex + i;
                int yCoor = static_cast<int>(curIndex / resolutionDimensionSize);
                int xCoor = static_cast<int>(curIndex - yCoor * resolutionDimensionSize);

                sampler.StartPixelSample({xCoor, yCoor}, sampleIndexOffset);
                int newVertices = TracePath(ray, graph, config.maxDepth, mediumExitTHit);

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

    OrderIdsAndRebuildTree(graph);

    if (config.reinforceSparseAreas)
        ReinforceSparseAreas(graph);

    if (config.addExtraEdges)
        AddExtraEdges(graph);

    UsePathInfo(graph);

    if (config.pruneLowDensity)
        PruneAndClean(graph);

    if (!quiet) {
        std::cout << StringPrintf("Vertices: %s, Edges: %s, Paths: %s",
            graph.GetVertices().size(), graph.GetEdges().size(), graph.GetPaths().size()) << std::endl;

        size_t numEdges = graph.GetEdges().size();
        size_t numEdgesMoreThanOnce = 0;
        for (auto& [id, edge] : graph.GetEdges()) {
            if (edge.data.samples > 1)
                ++numEdgesMoreThanOnce;
        }
        float moreThanOnceRatio = static_cast<float>(numEdgesMoreThanOnce) / static_cast<float>(numEdges);
        std::cout << StringPrintf("Edges connected more than once: %s / %s (%s)",
            numEdgesMoreThanOnce, numEdges, moreThanOnceRatio == 0 ? "-" : std::to_string(moreThanOnceRatio)) << std::endl;
        std::cout << StringPrintf("Edges added: %s", edgesAdded) << std::endl;

        util::Averager pathRemainLengthAverager;
        for (auto& [id, vertex] : graph.GetVertices())
            pathRemainLengthAverager.AddValue(vertex.data.pathRemainLength.value);

        std::cout << StringPrintf("Path remain length: %s", pathRemainLengthAverager.PrintInfo()) << std::endl;

        std::cout << StringPrintf("Scattered: in same sphere %s (corrected %s), scattered total %s, (%s)",
            scattersInSameSphere, scattersInSameSphereCorrected, totalScatters,
            static_cast<float>(scattersInSameSphereCorrected) / static_cast<float>(totalScatters)) << std::endl;
    }

    return graph;
}

std::vector<std::tuple<int, float>> FreeGraphBuilder::GetInRadius(const Point3f& pointRef, float squaredRadius, int vertexId) {
    std::vector<nanoflann::ResultItem<int, float>> resultItems;
    nanoflann::RadiusResultSet resultSet(squaredRadius, resultItems);
    float point[3] = {pointRef.x, pointRef.y, pointRef.z};

    searchTree->findNeighbors(resultSet, point);
    resultSet.sort();

    std::vector<std::tuple<int, float>> result;
    result.reserve(resultItems.size());

    for (nanoflann::ResultItem resultItem : resultItems) {
        if (resultItem.first != vertexId)
            result.emplace_back(std::tuple{resultItem.first, resultItem.second});
    }
    return result;
}

std::optional<std::tuple<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef, float squaredRadius, int vertexId) {
    std::vector<std::tuple<int, float>> result = GetInRadius(pointRef, squaredRadius, vertexId);
    return result.empty() ? std::nullopt : std::make_optional(result[0]);
}

inline int MoveEdgeReferences(Vertex& toMoveVertex, Vertex& targetVertex, Graph& graph) {
    struct TempEdge {
        TempEdge(int id, int from, int to, EdgeData data)
            : id(id), from(from), to(to), data(data) {
        }
        int id, from, to;
        EdgeData data;
    };

    int count = 0;

    std::vector<TempEdge> edgesToAdd;
    std::vector<int> edgesToRemove;

    for (auto& [fromId, edgeId] : toMoveVertex.inEdges) {
        Vertex& fromVertex = graph.GetVertex(fromId)->get();
        Edge& edge = graph.GetEdge(edgeId)->get();

        if (fromVertex.id == targetVertex.id) {
            edgesToRemove.push_back(edgeId);
            ++count;
        } else if (targetVertex.inEdges.find(fromVertex.id) == targetVertex.inEdges.end()) {
            edgesToAdd.emplace_back(edgeId, fromVertex.id, targetVertex.id, edge.data);
            edgesToRemove.push_back(edgeId);
        } else
            edgesToRemove.push_back(edgeId);
    }

    for (auto& [toId, edgeId] : toMoveVertex.outEdges) {
        Vertex& toVertex = graph.GetVertex(toId)->get();
        Edge& edge = graph.GetEdge(edgeId)->get();

        if (toVertex.id == targetVertex.id) {
            edgesToRemove.push_back(edgeId);
            ++count;
        } else if (targetVertex.outEdges.find(toVertex.id) == targetVertex.outEdges.end()) {
            edgesToAdd.emplace_back(edgeId, targetVertex.id, toVertex.id, edge.data);
            edgesToRemove.push_back(edgeId);
        } else
            edgesToRemove.push_back(edgeId);
    }

    for (int toRemoveId : edgesToRemove)
        graph.RemoveEdge(toRemoveId);

    for (TempEdge toAdd : edgesToAdd)
        graph.AddEdge(toAdd.id, toAdd.from, toAdd.to, toAdd.data);

    return count;
}

inline void MovePathReferences(Vertex& toMoveVertex, Vertex& targetVertex, Graph& graph) {
    for (auto& [pathId, indicesInPath] : toMoveVertex.paths) {
        Path& path = graph.GetPath(pathId)->get();
        for (int indexInPath : indicesInPath)
            path.vertices[indexInPath] = targetVertex.id;

        targetVertex.AddPathIndices(pathId, indicesInPath);
    }
    toMoveVertex.paths.clear();
}

inline int MoveVertexToTarget(Vertex& vertexToMove, Vertex& targetVertex, Graph& graph) {
    targetVertex.data.MergeWithDataFrom(vertexToMove.data);
    int count = MoveEdgeReferences(vertexToMove, targetVertex, graph);
    MovePathReferences(vertexToMove, targetVertex, graph);
    graph.RemoveVertex(vertexToMove.id);
    return count;
}

void FreeGraphBuilder::AddToTreeAndFit(Graph& graph, int startId, int endId) {
    searchTree->addPoints(startId, endId - 1); // method needs inclusive end range

    for (int curId = startId; curId < endId; ++curId) {
        std::optional<std::tuple<int, float>> resultItem = GetClosestInRadius(graph.GetVertex(curId)->get().point, squaredSearchRadius, curId);

        if (resultItem.has_value()) {
            Vertex& vertexToMove = graph.GetVertex(curId)->get();
            Vertex& targetVertex = graph.GetVertex(std::get<0>(resultItem.value()))->get();

            scattersInSameSphere += MoveVertexToTarget(vertexToMove, targetVertex, graph);

            searchTree->removePoint(curId);
        }
    }
}

void FreeGraphBuilder::UsePathInfo(Graph& graph) {
    ProgressReporter progress(static_cast<int64_t>(graph.GetVertices().size()), "Using path info", quiet);

    for (auto& [vertexId, vertex] : graph.GetVertices()) {
        util::Averager pathRemainLengthAverager;

        for (auto& [pathId, pathIndices] : vertex.paths) {
            if (pathIndices.empty())
                continue;

            float pathRemainLength = 1;

            for (int i = 1; i < pathIndices.size(); ++i) {
                if (pathIndices[i - 1] == pathIndices[i] - 1) {
                    --vertex.data.totalSamples;
                    ++scattersInSameSphereCorrected;
                    ++pathRemainLength;
                } else {
                    pathRemainLengthAverager.AddValue(pathRemainLength);
                    pathRemainLength = 1;
                }
            }
            pathRemainLengthAverager.AddValue(pathRemainLength);
        }
        vertex.data.pathRemainLength.FillWithAverager(pathRemainLengthAverager);

        progress.Update();
    }
    progress.Done();
}

void FreeGraphBuilder::AddExtraEdges(Graph& graph) {
    ProgressReporter progress(static_cast<int64_t>(graph.GetVertices().size()), "Adding extra edges", quiet);

    for (auto& [thisVertexId, thisVertex] : graph.GetVertices()) {
        for (auto [otherVertexId, edgeId] : thisVertex.outEdges) {
            Vertex& otherVertex = graph.GetVertex(otherVertexId)->get();
            Edge& edge = graph.GetEdge(edgeId)->get();
            if (otherVertex.outEdges.find(thisVertexId) == otherVertex.outEdges.end()) {
                graph.AddEdge(otherVertexId, thisVertexId, edge.data);
                ++edgesAdded;
            }
        }
        progress.Update();
    }
    progress.Done();
}

void FreeGraphBuilder::PruneAndClean(FreeGraph& graph) {
    if (!quiet)
        std::cout << "Pruning low PDF edges and sparse vertices... ";

    int verticesRemoved = 0;
    int vertexEdgesRemoved = 0;

    while (true) {
        std::vector<int> verticesToRemove;

        for (auto& [id, vertex] : graph.GetVertices())
            if (vertex.outEdges.size() < config.pruneVertexOutEdgesMinimum)
                verticesToRemove.push_back(id);

        for (int vertexId : verticesToRemove) {
            Vertex& vertex = graph.GetVertex(vertexId)->get();
            vertexEdgesRemoved += static_cast<int>(vertex.inEdges.size() + vertex.outEdges.size());

            graph.RemoveVertex(vertexId);
        }
        verticesRemoved += static_cast<int>(verticesToRemove.size());

        if (verticesToRemove.empty())
            break;
    }

    if (!quiet) {
        std::cout << "done" << std::endl;
        std::cout << StringPrintf("Removed %s sparse vertices (with %s edges)",
                verticesRemoved, vertexEdgesRemoved) << std::endl;
    }

    OrderIdsAndRebuildTree(graph);
}

void FreeGraphBuilder::ReinforceSparseAreas(FreeGraph& graph) {
    auto startTime = std::chrono::high_resolution_clock::now();
    if (!quiet)
        std::cout << "Searching for sparse areas... ";

    float neighbourSquaredSearchRadius = squaredSearchRadius * config.reinforcementRadiusModifierMult;

    std::vector<int> sparseVertices;
    for (auto& [id, vertex] : graph.GetVertices()) {
        std::vector<std::tuple<int, float>> result = GetInRadius(vertex.point, neighbourSquaredSearchRadius, id);

        if (result.size() < config.neighboursForNotSparse)
            sparseVertices.push_back(id);
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    if (!quiet) {
        std::cout << StringPrintf("done (%ss)", duration) << std::endl;
        std::cout << StringPrintf("%s / %s vertices found with few neighbours", sparseVertices.size(), graph.GetVertices().size()) << std::endl;
    }

    float sphereRadius = graph.GetVertexRadius().value();
    util::SpherePointsMaker spherePointsMaker(sphereRadius, config.pointsOnRadiusReinforcement);

    int64_t numSparseVertices = static_cast<int64_t>(sparseVertices.size());
    int numRaysPerVertex = config.reinforcementIterations * util::GetSphereVolumePointsSize(config.pointsOnRadiusReinforcement);
    int64_t workNeeded = numSparseVertices * numRaysPerVertex;

    ProgressReporter progress(workNeeded, "Reinforcing sparse areas", quiet);
    int resolutionDimensionSize = Options->graph.samplingResolution->x;

    int batchSize = 100;
    int startingId = graph.GetCurVertexId();
    int currentBatch = 0;

    for (int vertexId : sparseVertices) {
        Vertex& vertex = graph.GetVertex(vertexId)->get();
        std::vector<Point3f> spherePoints = spherePointsMaker.GetSpherePointsFor(vertex.point);

        uint64_t startIndex = vertexId * static_cast<int>(spherePoints.size());
        for (int pointIndex = 0; pointIndex < spherePoints.size(); ++pointIndex) {
            uint64_t curIndex = startIndex + pointIndex;
            int yCoor = static_cast<int>(curIndex / resolutionDimensionSize);
            int xCoor = static_cast<int>(curIndex - yCoor * resolutionDimensionSize);

            Point3f spherePoint = spherePoints[pointIndex];
            PhaseFunction phase = mediumData.medium.SamplePoint(spherePoint, mediumData.defaultLambda).phase;

            for (int i = 0; i < config.reinforcementIterations; ++i) {
                sampler.StartPixelSample({xCoor, yCoor}, i);

                Vector3f inDir(1, 0, 0);
                Vector3f outDir = phase.Sample_p(inDir, sampler.Get2D())->wi;
                RayDifferential ray(spherePoint, outDir, 0, mediumData.medium);
                util::HitsResult mediumHits = GetHits(mediumData.primitiveData.primitive, ray, mediumData);

                if (!mediumHits.RayEntersVolume()) {
                    progress.Update();
                    continue;
                }

                if (mediumHits.type == util::OutsideTwoHits)
                    mediumHits.intersections[0].intr.SkipIntersection(&ray, mediumHits.tHits[0]);

                float mediumExitTHit = mediumHits.type == util::OutsideTwoHits ? mediumHits.tHits[1] - mediumHits.tHits[0] : mediumHits.tHits[0];

                int newVertices = TracePath(ray, graph, config.maxDepth, mediumExitTHit);
                currentBatch += newVertices;

                if (currentBatch >= batchSize) {
                    AddToTreeAndFit(graph, startingId, startingId + currentBatch);

                    startingId += currentBatch;
                    currentBatch = 0;
                }
                progress.Update();
            }
        }
    }
    AddToTreeAndFit(graph, startingId, startingId + currentBatch);
    progress.Done();

    int sparseVerticesAfter = 0;
    for (int vertexId : sparseVertices) {
        Vertex& vertex = graph.GetVertex(vertexId)->get();
        std::vector<std::tuple<int, float>> result = GetInRadius(vertex.point, neighbourSquaredSearchRadius, vertexId);

        if (result.size() < config.neighboursForNotSparse)
            ++sparseVerticesAfter;
    }

    if (!quiet)
        std::cout << StringPrintf("%s / %s sparse vertices after reinforcing", sparseVerticesAfter, numSparseVertices) << std::endl;

    OrderIdsAndRebuildTree(graph);
}

void FreeGraphBuilder::OrderIdsAndRebuildTree(Graph& graph) {
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

    graph.SetCurVertexId(currentLargestId + 1);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    if (!quiet)
        std::cout << StringPrintf("done (%ss)", duration) << std::endl;

    graph.CheckSequentialIds();

    startTime = std::chrono::high_resolution_clock::now();
    if (!quiet)
        std::cout << "Rebuilding search tree... ";

    int numVertices = static_cast<int>(graph.GetVertices().size());

    std::vector<Point3f>& list = vHolder.GetList();
    int sizeBefore = static_cast<int>(list.size());
    list.clear();
    list.shrink_to_fit();
    list.reserve(numVertices);

    searchTree = std::make_unique<DynamicTreeType>(3, vHolder);

    if (numVertices == 0)
        return;

    for (int id = 0; id < numVertices; ++id) {
        Vertex& vertex = graph.GetVertex(id)->get();
        list.push_back(vertex.point);
    }
    int sizeAfter = static_cast<int>(list.size());

    int batchSize = 1000;
    int lastId = numVertices - 1;
    for (int startingId = 0; startingId < numVertices; startingId += batchSize) {
        int endingId = std::min(startingId + batchSize - 1, lastId);
        searchTree->addPoints(startingId, endingId);
    }

    endTime = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    if (!quiet)
        std::cout << StringPrintf("done (%ss, %s to %s vertices)", duration, sizeBefore, sizeAfter) << std::endl;
}
}
