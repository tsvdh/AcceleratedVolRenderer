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
        std::optional<std::tuple<int, float>> optResult = GetClosestInRadius(newPoint, squaredSearchRadius);
        std::optional<Point3f> prevPoint = path.vertices.empty()
            ? std::nullopt
            : std::make_optional(graph.GetVertex(path.vertices.back())->get().point);

        Vertex* newVertex;
        if (optResult) {
            newVertex = &graph.GetVertex(std::get<0>(optResult.value()))->get();
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
        if (path.vertices.size() == maxDepth) {
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

    if (!quiet) {
        std::cout << "=== Graph stats ===" << std::endl;
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

        std::cout << StringPrintf("Path remain length: %s", inNodePathLengthAverager.PrintInfo()) << std::endl;
        std::cout << "===================" << std::endl;
    }

    return graph;
}

std::vector<std::tuple<int, float>> FreeGraphBuilder::GetInRadius(const Point3f& pointRef, float squaredRadius) {
    std::vector<nanoflann::ResultItem<int, float>> resultItems;
    nanoflann::RadiusResultSet resultSet(squaredRadius, resultItems);
    float point[3] = {pointRef.x, pointRef.y, pointRef.z};

    searchTree->findNeighbors(resultSet, point);
    resultSet.sort();

    std::vector<std::tuple<int, float>> result;
    result.reserve(resultItems.size());

    for (nanoflann::ResultItem resultItem : resultItems)
        result.emplace_back(std::tuple{resultItem.first, resultItem.second});

    return result;
}

std::optional<std::tuple<int, float>> FreeGraphBuilder::GetClosestInRadius(const Point3f& pointRef, float squaredRadius) {
    std::vector<std::tuple<int, float>> result = GetInRadius(pointRef, squaredRadius);
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
            inNodePathLengthAverager.AddValue(1);
            continue;
        }

        float inNodePathLength = 1;

        for (int i = 0; i < path.vertices.size() - 1; ++i) {
            if (path.vertices[i] == path.vertices[i + 1]) {
                ++inNodePathLength;
            } else {
                inNodePathLengthAverager.AddValue(inNodePathLength);
                inNodePathLength = 1;
            }
        }
        if (!path.data.forcedEnd)
            inNodePathLengthAverager.AddValue(inNodePathLength);
    }

    for (int pathId : pathsToRemove)
        graph.RemovePath(pathId);
}

void FreeGraphBuilder::ReinforceSparseVertices(FreeGraph& graph) {
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
                currentFewEdges.size(), initialVertices.size(), edgesUnsatisfiedRatio, util::formatTime(edgeDuration)) << std::endl;
    };
    auto printNeighbourStatus = [&]() -> void {
        if (quiet)
            return;
        if (!config.neighbourReinforcement.active)
            std::cout << "Neighbours: inactive" << std::endl;
        else
            std::cout << StringPrintf("Neighbours: %s / %s; %.3f (%s)          ",
                currentFewNeighbours.size(), initialVertices.size(), neighboursUnsatisfiedRatio, util::formatTime(neighbourDuration)) << std::endl;
    };
    auto clearLine = []() -> void {
        std::string blankLine;
        blankLine.assign(TerminalWidth(), ' ');
        std::cout << blankLine << "\r";
    };

    auto checkFewEdges = [&]() -> void {
        currentFewEdges.clear();
        for (int id : initialVertices) {
            Vertex& vertex = graph.GetVertex(id)->get();
            if (vertex.outEdges.size() < config.edgeReinforcement.edgesForNotSparse)
                currentFewEdges.push_back(id);
        }

        edgesUnsatisfiedRatio = static_cast<float>(currentFewEdges.size()) / static_cast<float>(initialVertices.size());
        edgesSatisfied = edgesUnsatisfiedRatio < config.edgeReinforcement.unsatisfiedAllowedRatio;
    };

    auto checkFewNeighbours = [&]() -> void {
        currentFewNeighbours.clear();
        for (int id : initialVertices) {
            Vertex& vertex = graph.GetVertex(id)->get();
            if (CountInRadius(vertex.point, squaredNeighbourSearchRadius) < config.neighbourReinforcement.neighboursForNotSparse)
                currentFewNeighbours.push_back(id);
        }

        neighboursUnsatisfiedRatio = static_cast<float>(currentFewNeighbours.size()) / static_cast<float>(initialVertices.size());
        neighboursSatisfied = neighboursUnsatisfiedRatio < config.neighbourReinforcement.unsatisfiedAllowedRatio;
    };

    for (auto& [id, vertex] : graph.GetVertices())
        initialVertices.push_back(id);

    if (config.edgeReinforcement.active)
        checkFewEdges();
    if (config.neighbourReinforcement.active)
        checkFewNeighbours();

    printEdgeStatus();
    printNeighbourStatus();

    int cycle = 0;
    while (!edgesSatisfied || !neighboursSatisfied) {
        if (config.edgeReinforcement.active && !edgesSatisfied) {

            ProgressReporter progress(currentFewEdges.size() * config.edgeReinforcement.reinforcementRays,
                "Reinforcing edges", false);

            ReinforceSparseVertices(graph, currentFewEdges, config.edgeReinforcement, cycle, progress);
            edgeDuration += progress.ElapsedSeconds();
            checkFewEdges();

            std::cout << "\x1b[3A";
            printEdgeStatus();
            printNeighbourStatus();
            clearLine();
        }
        if (config.neighbourReinforcement.active && !neighboursSatisfied) {

            ProgressReporter progress(currentFewNeighbours.size() * config.neighbourReinforcement.reinforcementRays,
                "Reinforcing neighbours", false);

            ReinforceSparseVertices(graph, currentFewNeighbours, config.neighbourReinforcement, cycle, progress);
            neighbourDuration += progress.ElapsedSeconds();
            checkFewNeighbours();

            std::cout << "\x1b[3A";
            printEdgeStatus();
            printNeighbourStatus();
            clearLine();
        }
        ++cycle;
    }

    if (!quiet) {
        std::cout << StringPrintf("Done (%s)                         ",
            util::formatTime(edgeDuration + neighbourDuration)) << std::endl;
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
            graph.GetVertexRadius().value(), vertex.point, reinforcementConfig.reinforcementRays, sampler);

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

            TracePath(ray, graph, config.maxDepth, mediumExitTHit, vertexId);
            UseAndRemovePathInfo(graph);

            progress.Update();
        }
    }
    progress.Done();
}

}
