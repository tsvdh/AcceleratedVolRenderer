#include "lighting_calculator.h"

#include "pbrt/lights.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
LightingCalculator::LightingCalculator(FreeGraph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
                                       const LightingCalculatorConfig& config, bool quiet)
    : graph(graph), mediumData(mediumData), inDirection(inDirection), sampler(std::move(sampler)), config(config), quiet(quiet) {
    if (config.lightIterations <= 0)
        ErrorExit("Must have at least one light ray iteration");

    graph.CheckSequentialIds();
    numVertices = static_cast<int>(graph.GetVerticesConst().size());
}

int LightingCalculator::ComputeFinalLight(int bouncesIndex) {
    SparseVec initialLight = GetLightVector();
    SparseMat transportMatrix = GetTransportMatrix();
    return ComputeFinalLight(initialLight, transportMatrix, bouncesIndex);
}

int LightingCalculator::ComputeFinalLight(const SparseVec& lightVec, const SparseMat& transportMat, int bouncesIndex) {
    int bounces = config.bounces[bouncesIndex];

    SparseVec totalLight(lightVec);

    int curIteration = 0;

    if (bounces > 0) {
        SparseVec curLight = totalLight;

        ProgressReporter transportProgress(bounces, "Computing final lighting", quiet);

        for (; curIteration < bounces; ++curIteration) {
            curLight = transportMat * curLight;

            bool invalidNumbers = false;
            for (int i = 0; i < numVertices; ++i) {
                if (IsNaN(curLight.coeff(i)) || IsInf(curLight.coeff(i))) {
                    invalidNumbers = true;
                    break;
                }
            }
            if (invalidNumbers)
                break;

            totalLight += curLight;

            transportProgress.Update();
        }
        transportProgress.Done();
    }

    for (auto& [id, vertex] : graph.GetVertices())
        vertex.data.lightScalar = totalLight.coeff(id);

    return curIteration;
}

SparseMat LightingCalculator::GetTransportMatrix() const {
    ProgressReporter progress(numVertices, "Computing edge weights", quiet);

    std::vector<Eigen::Triplet<float>> entries;
    entries.reserve(graph.GetEdges().size());

    for (auto& [currentVertexId, vertex] : graph.GetVertices()) {
        int samples = vertex.data.samples;

        for (auto& [otherVertexId, edgeId] : vertex.outEdges) {
            int edgeSamples = graph.GetEdge(edgeId)->get().data.samples;
            entries.emplace_back(otherVertexId, currentVertexId,
                static_cast<float>(edgeSamples) / static_cast<float>(samples));
        }
        progress.Update();
    }
    progress.Done();

    SparseMat transportMatrix(numVertices, numVertices);
    transportMatrix.setFromTriplets(entries.begin(), entries.end());
    transportMatrix = transportMatrix.transpose();
    return transportMatrix;
}

SparseVec LightingCalculator::GetLightVector() {
    std::unordered_map<int, float> lightMap;
    std::vector<int> vertexIds;
    lightMap.reserve(numVertices);
    vertexIds.reserve(numVertices);

    for (auto& [id, vertex] : graph.GetVertices()) {
        lightMap[id] = 0; // allocate space for concurrent access
        vertexIds.push_back(id);
    }

    ThreadLocal<Sampler> samplers([&] { return sampler.Clone(); });

    float sphereRadius = graph.GetVertexRadius().value();
    util::SphereMaker sphereMaker(sphereRadius);

    int64_t workNeeded = numVertices * config.lightIterations * util::GetDiskPointsSize(config.pointsOnRadiusLight);
    ProgressReporter progress(workNeeded, "Computing initial lighting", quiet);

    ParallelFor(0, numVertices, config.runInParallel, [&](int listIndex) {
        Sampler& samplerClone = samplers.Get();

        int vertexId = vertexIds[listIndex];
        Vertex& vertex = graph.GetVertex(vertexId)->get();

        SphereContainer currentSphere = sphereMaker.GetSphereFor(vertex.point);

        Point3f origin = vertex.point - inDirection * mediumData.primitiveData.maxDistToCenter * 2;
        std::vector<Point3f> diskPoints = util::GetDiskPoints(origin, sphereRadius, config.pointsOnRadiusLight, inDirection);

        util::Averager transmittanceAverager;

        uint64_t startIndex = listIndex * diskPoints.size();
        for (int pointIndex = 0; pointIndex < diskPoints.size(); ++pointIndex) {
            uint64_t curIndex = startIndex + pointIndex;

            Point3f diskPoint = diskPoints[pointIndex];
            RayDifferential rayToSphere(diskPoint, inDirection, 0, mediumData.medium);

            util::HitsResult mediumHits = GetHits(mediumData.primitiveData.primitive, rayToSphere, mediumData);
            util::HitsResult sphereHits = GetHits(currentSphere.sphere, rayToSphere, mediumData);

            if (mediumHits.type != util::OutsideTwoHits || sphereHits.type != util::OutsideTwoHits) {
                progress.Update(config.lightIterations);
                continue;
            }

            util::StartEndT startEnd = GetStartEndT(mediumHits, sphereHits);

            if (startEnd.sphereRayNotInMedium) {
                progress.Update(config.lightIterations);
                continue;
            }

            mediumHits.intersections[0].intr.SkipIntersection(&rayToSphere, startEnd.startT);
            startEnd.SkipForward(startEnd.startT);

            transmittanceAverager.AddValue(ComputeRaysToSphere(rayToSphere, std::nullopt, startEnd, mediumData, samplerClone,
                config.lightIterations, curIndex));

            progress.Update(config.lightIterations);
        }

        lightMap[vertexId] = transmittanceAverager.GetAverage();
    });
    progress.Done();

    for (auto& [id, light] : lightMap)
        light *= Inv4Pi;

    return LightMapToVector(lightMap);
}

SparseVec LightingCalculator::LightMapToVector(const std::unordered_map<int, float>& lightMap) const {
    std::vector<std::pair<int, float>> lightPairs(lightMap.begin(), lightMap.end());
    std::sort(lightPairs.begin(), lightPairs.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    SparseVec lightVector(numVertices);
    for (auto& pair : lightPairs) {
        lightVector.coeffRef(pair.first) = pair.second;
    }

    return lightVector;
}
}
