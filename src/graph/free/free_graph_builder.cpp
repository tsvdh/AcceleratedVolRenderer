#include "free_graph_builder.h"

#include <iostream>
#include <set>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/options.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, const GraphBuilderConfig& config,
                                   bool quiet, int sampleIndexOffset)
    : FreeGraphBuilder(mediumData, inDirection, std::move(sampler), config, quiet, sampleIndexOffset,
                       Sqr(GetSameSpotRadius(mediumData) * config.radiusModifier),
                       Sqr(GetSameSpotRadius(mediumData) * config.neighbourReinforcement.neighbourRangeModifier)) {
}

FreeGraphBuilder::FreeGraphBuilder(const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler, const GraphBuilderConfig& config,
                                   bool quiet, int sampleIndexOffset, float squaredSearchRadius, float squaredNeighbourSearchRadius)
    : mediumData(mediumData), inDirection(inDirection), sampler(std::move(sampler)), config(config), quiet(quiet),
      squaredSearchRadius(squaredSearchRadius), squaredNeighbourSearchRadius(squaredNeighbourSearchRadius), sampleIndexOffset(sampleIndexOffset) {
    searchTree = std::make_unique<DynamicTreeType>(3, vHolder);
}

void FreeGraphBuilder::TracePath(RayDifferential ray, FreeGraph& graph, int maxDepth, float firstSegmentTHit, std::optional<int> startingVertex) {
    int numNewVertices = 0;
    Path& path = graph.AddPath(PathData{});
    bool usedTHit = false;

    if (startingVertex.has_value())
        graph.AddVertexToPath(startingVertex.value(), path.id);

    auto HandlePotentialPathEnd = [&] {
        if (!path.vertices.empty()) {
            Vertex& lastVertex = graph.GetVertex(path.vertices.back())->get();
            ++lastVertex.data.samples;
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
                return;
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
            return;
        }

        Point3f newPoint = optNewInteraction->p();
        std::optional<nanoflann::ResultItem<int, float>> optResult = GetClosestInRadius(newPoint, squaredSearchRadius);
        std::optional<Point3f> prevPoint = path.vertices.empty()
            ? std::nullopt
            : std::make_optional(graph.GetVertex(path.vertices.back())->get().point);

        Vertex* newVertex;
        if (optResult) {
            newVertex = &graph.GetVertex(optResult.value().first)->get();
        }
        else if (prevPoint && DistanceSquared(prevPoint.value(), newPoint) <= squaredSearchRadius) {
            newVertex = &graph.GetVertex(path.vertices.back())->get();
        }
        else {
            newVertex = &graph.AddVertex(newPoint, VertexData{});
            vHolder.GetList().emplace_back(newVertex->point);
            searchTree->addPoints(newVertex->id, newVertex->id);
            ++numNewVertices;
        }

        graph.AddVertexToPath(newVertex->id, path.id);

        if (path.vertices.size() >= 2) {
            int pathSize = path.Length();
            int from = path.vertices[pathSize - 2];
            int to = path.vertices[pathSize - 1];

            graph.AddEdge(from, to, EdgeData{1});
        }

        // terminate if max depth reached
        int newPathVertices = static_cast<int>(path.vertices.size());
        if (startingVertex.has_value())
            --newPathVertices;
        if (newPathVertices >= maxDepth) {
            // pathLengths.push_back(path.size());
            path.data.forcedEnd = true;
            return;
        }

        // Sample new direction at real-scattering event
        Point2f u = sampler.Get2D();
        pstd::optional<PhaseFunctionSample> ps = optNewInteraction->phase.Sample_p(-ray.d, u);

        ray.o = newPoint;
        ray.d = ps->wi;
    }
}

FreeGraph FreeGraphBuilder::BuildGraph() {
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
                TracePath(ray, graph, config.maxDepth, mediumExitTHit, std::nullopt);
                UseAndRemovePathInfo(graph);

                tracingProgress.Update();
            }
        }
    }
    tracingProgress.Done();

    ReinforceSparseVertices(graph);
    ComputeSearchRanges(graph);

    if (!quiet) {
        std::cout << "=== Graph stats ===" << std::endl;
        json stats;
        graph.AddStats(stats);
        stats["node_radius"] = graph.GetVertexRadius();
        std::cout << std::setw(2) << stats << std::endl;
        std::cout << "===================" << std::endl;
    }

    return graph;
}

std::vector<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetInRadius(const Point3f& pointRef, float squaredRadius) {
    std::vector<nanoflann::ResultItem<int, float>> resultItems;
    nanoflann::RadiusResultSet resultSet(squaredRadius, resultItems);
    float point[3] = {pointRef.x, pointRef.y, pointRef.z};

    searchTree->findNeighbors(resultSet, point);

    return resultItems;
}

std::optional<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef, float squaredRadius) {
    std::vector<nanoflann::ResultItem<int, float>> result = GetInRadius(pointRef, squaredRadius);
    return result.empty() ? std::nullopt : std::make_optional(result[0]);
}

int FreeGraphBuilder::CountInRadius(const Point3f& pointRef, float squaredRadius) {
    std::vector<nanoflann::ResultItem<int, float>> resultItems;
    nanoflann::RadiusResultSet resultSet(squaredRadius, resultItems);
    float point[3] = {pointRef.x, pointRef.y, pointRef.z};

    searchTree->findNeighbors(resultSet, point);
    return static_cast<int>(resultItems.size());
}

void FreeGraphBuilder::UseAndRemovePathInfo(Graph& graph) {
    std::vector<int> pathsToRemove;
    pathsToRemove.reserve(graph.GetPaths().size());

    for (auto& [pathId, path] : graph.GetPaths()) {
        pathsToRemove.push_back(pathId);

        if (path.Length() == 0) {
            continue;
        }

        if (path.Length() == 1 && !path.data.forcedEnd) {
            graph.inNodePathLengthAverager.AddValue(1);
            continue;
        }

        float inNodePathLength = 1;

        for (int i = 0; i < path.vertices.size() - 1; ++i) {
            if (path.vertices[i] == path.vertices[i + 1]) {
                ++inNodePathLength;
            } else {
                graph.inNodePathLengthAverager.AddValue(inNodePathLength);
                inNodePathLength = 1;
            }
        }
        if (!path.data.forcedEnd)
            graph.inNodePathLengthAverager.AddValue(inNodePathLength);
    }

    for (int pathId : pathsToRemove)
        graph.RemovePath(pathId);
}

inline void ClearLine() {
    std::string blankLine;
    blankLine.assign(TerminalWidth(), ' ');
    std::cout << blankLine << "\r";
}

void FreeGraphBuilder::ReinforceSparseVertices(FreeGraph& graph) {
    if (!config.edgeReinforcement.active && !config.neighbourReinforcement.active)
        return;
    if (!quiet)
        std::cout << "--- Graph Reinforcement ---" << std::endl;

    bool edgesSatisfied = true, neighboursSatisfied = true;
    float edgesUnsatisfiedRatio = 0, neighboursUnsatisfiedRatio = 0;
    std::vector<int> initialVertices;
    std::vector<int> currentFewEdges, currentFewNeighbours;

    float edgeDuration = 0, neighbourDuration = 0;

    auto printEdgeStatus = [&]() -> void {
        if (quiet)
            return;
        if (!config.edgeReinforcement.active)
            std::cout << "Edges: inactive" << std::endl;
        else
            std::cout << StringPrintf("Edges: %s / %s; %.3f (%s)               ",
                currentFewEdges.size(), initialVertices.size(), edgesUnsatisfiedRatio, util::FormatTime(edgeDuration)) << std::endl;
    };
    auto printNeighbourStatus = [&]() -> void {
        if (quiet)
            return;
        if (!config.neighbourReinforcement.active)
            std::cout << "Neighbours: inactive" << std::endl;
        else
            std::cout << StringPrintf("Neighbours: %s / %s; %.3f (%s)          ",
                currentFewNeighbours.size(), initialVertices.size(), neighboursUnsatisfiedRatio, util::FormatTime(neighbourDuration)) << std::endl;
    };

    auto checkFewEdges = [&]() -> void {
        ProgressReporter progress(initialVertices.size(), "Checking edges", quiet);

        currentFewEdges.clear();
        for (int id : initialVertices) {
            Vertex& vertex = graph.GetVertex(id)->get();
            if (vertex.outEdges.size() < config.edgeReinforcement.edgesForNotSparse)
                currentFewEdges.push_back(id);
            progress.Update();
        }
        progress.Done();
        if (!quiet) {
            std::cout << "\x1b[1A";
            ClearLine();
        }

        edgesUnsatisfiedRatio = static_cast<float>(currentFewEdges.size()) / static_cast<float>(initialVertices.size());
        edgesSatisfied = edgesUnsatisfiedRatio < config.edgeReinforcement.unsatisfiedAllowedRatio;
    };

    auto checkFewNeighbours = [&]() -> void {
        ProgressReporter progress(initialVertices.size(), "Checking neighbours", quiet);

        currentFewNeighbours.clear();
        for (int id : initialVertices) {
            Vertex& vertex = graph.GetVertex(id)->get();
            if (CountInRadius(vertex.point, squaredNeighbourSearchRadius) < config.neighbourReinforcement.neighboursForNotSparse)
                currentFewNeighbours.push_back(id);
            progress.Update();
        }
        progress.Done();
        if (!quiet) {
            std::cout << "\x1b[1A";
            ClearLine();
        }

        neighboursUnsatisfiedRatio = static_cast<float>(currentFewNeighbours.size()) / static_cast<float>(initialVertices.size());
        neighboursSatisfied = neighboursUnsatisfiedRatio < config.neighbourReinforcement.unsatisfiedAllowedRatio;
    };

    for (auto& [id, vertex] : graph.GetVertices())
        initialVertices.push_back(id);

    if (config.edgeReinforcement.active)
        checkFewEdges();
    printEdgeStatus();

    if (config.neighbourReinforcement.active)
        checkFewNeighbours();
    printNeighbourStatus();

    int cycle = 0;
    while (!edgesSatisfied || !neighboursSatisfied) {
        if (config.edgeReinforcement.active && !edgesSatisfied) {

            ProgressReporter progress(currentFewEdges.size() * config.edgeReinforcement.reinforcementRays,
                "Reinforcing edges", false);

            ReinforceSparseVertices(graph, currentFewEdges, config.edgeReinforcement, cycle, progress);
            edgeDuration += progress.ElapsedSeconds();

            if (!quiet) {
                checkFewEdges();

                std::cout << "\x1b[2A";
                printEdgeStatus();
                printNeighbourStatus();
            }
        }
        if (config.neighbourReinforcement.active && !neighboursSatisfied) {

            ProgressReporter progress(currentFewNeighbours.size() * config.neighbourReinforcement.reinforcementRays,
                "Reinforcing neighbours", false);

            ReinforceSparseVertices(graph, currentFewNeighbours, config.neighbourReinforcement, cycle, progress);
            neighbourDuration += progress.ElapsedSeconds();

            if (!quiet) {
                checkFewNeighbours();

                std::cout << "\x1b[2A";
                printEdgeStatus();
                printNeighbourStatus();
            }
        }
        ++cycle;
    }

    if (!quiet) {
        std::cout << StringPrintf("Done (%s)                         ",
            util::FormatTime(edgeDuration + neighbourDuration)) << std::endl;
        std::cout << "---------------------------" << std::endl;
    }
}

void FreeGraphBuilder::ReinforceSparseVertices(FreeGraph& graph, const std::vector<int>& sparseVertices, const ReinforcementConfig& reinforcementConfig,
        int cycle, ProgressReporter& progress) {
    int resolutionDimensionSize = Options->graph.samplingResolution->x;

    for (int vertexId : sparseVertices) {
        Vertex& vertex = graph.GetVertex(vertexId)->get();

        sampler.StartPixelSample(Point2i(0, 0), cycle);
        std::vector<Point3f> spherePoints = util::GetSphereVolumePointsRandom(
            graph.GetVertexRadius(), vertex.point, reinforcementConfig.reinforcementRays, sampler);

        uint64_t startIndex = vertexId * static_cast<int>(spherePoints.size());
        for (int pointIndex = 0; pointIndex < spherePoints.size(); ++pointIndex) {
            uint64_t curIndex = startIndex + pointIndex;
            int yCoor = static_cast<int>(curIndex / resolutionDimensionSize);
            int xCoor = static_cast<int>(curIndex - yCoor * resolutionDimensionSize);

            Point3f spherePoint = spherePoints[pointIndex];
            PhaseFunction phase = mediumData.medium.SamplePoint(spherePoint, mediumData.defaultLambda).phase;
            Vector3f inDir(1, 0, 0);

            sampler.StartPixelSample({xCoor, yCoor}, cycle);

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

            TracePath(ray, graph, 1, mediumExitTHit, vertexId);
            UseAndRemovePathInfo(graph);

            progress.Update();
        }
    }
    progress.Done();

    std::cout << "\x1b[1A";
    ClearLine();
}

std::vector<nanoflann::ResultItem<int, float>> FreeGraphBuilder::GetNClosest(const Point3f& pointRef, int nClosest) {
    // find this vertex and n closest
    ++nClosest;

    std::vector<size_t> indices;
    std::vector<float> distSqr;
    indices.resize(nClosest);
    distSqr.resize(nClosest);
    nanoflann::KNNResultSet<float> resultSet(nClosest);
    resultSet.init(indices.data(), distSqr.data());

    float point[3] = {pointRef.x, pointRef.y, pointRef.z};

    searchTree->findNeighbors(resultSet, point);
    resultSet.sort();

    std::vector<nanoflann::ResultItem<int, float>> result;
    result.reserve(nClosest - 1);

    for (int i = 1; i < resultSet.size(); ++i)
        result.emplace_back(indices[i], distSqr[i]);

    return result;
}

void FreeGraphBuilder::ComputeSearchRanges(FreeGraph& graph) {
    int nClosest = config.renderSearchRangeConfig.neighboursToUse;
    int numVertices = static_cast<int>(graph.GetVertices().size());

    if (nClosest > numVertices)
        ErrorExit("Graph does not contain enough vertices");

    int extraWork = static_cast<int>(numVertices * 1.01) - numVertices;
    ProgressReporter progress(numVertices + extraWork, "Computing search ranges", quiet);

    std::vector<float> avgDistToNeighbours;
    std::vector<std::vector<int>> neighbours;
    avgDistToNeighbours.assign(numVertices, -1);
    neighbours.assign(numVertices, std::vector(nClosest, -1));

    ParallelFor(0, numVertices, config.renderSearchRangeConfig.runInParallel, [&](int index) {
        Vertex& vertex = graph.GetVertex(index)->get();

        util::Averager distAverager(nClosest);

        std::vector<nanoflann::ResultItem<int, float>> nClosestVertices = GetNClosest(vertex.point, nClosest);
        for (int i = 0; i < nClosestVertices.size(); ++i) {
            nanoflann::ResultItem<int, float> resultItem = nClosestVertices[i];
            distAverager.AddValue(sqrt(resultItem.second));
            neighbours[index][i] = resultItem.first;
        }
        avgDistToNeighbours[index] = distAverager.GetAverage();
        progress.Update();
    });

    for (int i = 0; i < numVertices; ++i) {
        Vertex& vertex = graph.GetVertex(i)->get();

        util::Averager neighbourDistAverager(nClosest);
        neighbourDistAverager.AddValue(avgDistToNeighbours[i]);

        for (int neighbourId : neighbours[i]) {
            neighbourDistAverager.AddValue(avgDistToNeighbours[neighbourId]);
        }

        vertex.data.renderSearchRange = neighbourDistAverager.GetAverage();
    }

    progress.Update(extraWork);
    progress.Done();
}

}
