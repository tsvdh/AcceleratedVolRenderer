#include "free_lighting_calculator.h"

#include "pbrt/lights.h"
#include "pbrt/options.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeLightingCalculator::FreeLightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
        LightingCalculatorConfig config, bool quiet, int sampleIndexOffset)
    : LightingCalculator(graph, mediumData, inDirection, std::move(sampler), config, quiet, sampleIndexOffset) {

    freeGraph = dynamic_cast<FreeGraph*>(&graph);
}

SparseVec FreeLightingCalculator::GetLightVector() {
    int64_t numVertices = static_cast<int64_t>(graph.GetVertices().size());

    std::unordered_map<int, float> lightMap;
    std::vector<int> vertexIds;
    lightMap.reserve(numVertices);
    vertexIds.reserve(numVertices);

    for (auto& [id, vertex] : graph.GetVertices()) {
        lightMap[id] = 0; // allocate space for concurrent access
        vertexIds.push_back(id);
    }

    ThreadLocal<Sampler> samplers([&] { return sampler.Clone(); });
    ThreadLocal<ScratchBuffer> scratchBuffers([] { return ScratchBuffer(); });

    float sphereRadius = freeGraph->GetVertexRadius().value();
    util::SphereMaker sphereMaker(sphereRadius);

    int64_t workNeeded = numVertices * config.lightRayIterations * util::GetDiskPointsSize(config.pointsOnRadius);
    ProgressReporter progress(workNeeded, "Computing initial lighting", quiet);

    ParallelFor(0, numVertices, config.runInParallel, [&](int listIndex) {
        Sampler& samplerClone = samplers.Get();
        ScratchBuffer& buffer = scratchBuffers.Get();

        int vertexId = vertexIds[listIndex];
        Vertex& vertex = graph.GetVertex(vertexId)->get();

        SphereContainer currentSphere = sphereMaker.GetSphereFor(vertex.point);

        Point3f origin = vertex.point - inDirection * mediumData.primitiveData.maxDistToCenter * 2;
        std::vector<Point3f> diskPoints = util::GetDiskPoints(origin, sphereRadius, config.pointsOnRadius, inDirection);

        util::Averager transmittanceAverager;

        uint64_t startIndex = listIndex * diskPoints.size();
        for (int pointIndex = 0; pointIndex < diskPoints.size(); ++pointIndex) {
            uint64_t curIndex = startIndex + pointIndex;

            Point3f diskPoint = diskPoints[pointIndex];
            RayDifferential rayToSphere(diskPoint, inDirection, 0, mediumData.medium);

            util::HitsResult mediumHits = GetHits(mediumData.primitiveData.primitive, rayToSphere, mediumData);
            util::HitsResult sphereHits = GetHits(currentSphere.sphere, rayToSphere, mediumData);

            if (mediumHits.type != util::OutsideTwoHits || sphereHits.type != util::OutsideTwoHits) {
                progress.Update(config.lightRayIterations);
                continue;
            }

            util::StartEndT startEnd = GetStartEndT(mediumHits, sphereHits);

            if (startEnd.sphereRayNotInMedium) {
                progress.Update(config.lightRayIterations);
                continue;
            }

            mediumHits.intersections[0].intr.SkipIntersection(&rayToSphere, startEnd.startT);
            startEnd.SkipForward(startEnd.startT);

            transmittanceAverager.AddValue(ComputeRaysScatteredInSphere(rayToSphere, startEnd, mediumData, samplerClone, buffer,
                config.lightRayIterations, curIndex));

            progress.Update(config.lightRayIterations);
        }

        lightMap[vertexId] = transmittanceAverager.GetAverage();
    });
    progress.Done();

    return LightMapToVector(lightMap);
}

SparseMat FreeLightingCalculator::GetGMatrix() const {
    auto& edges = graph.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> gEntries;
    gEntries.reserve(edges.size());

    for (auto& [id, edge] : edges) {
        float T = edge.data.throughput;
        gEntries.emplace_back(edge.to, edge.from, T);
    }

    SparseMat gMatrix(numVertices, numVertices);
    gMatrix.setFromTriplets(gEntries.begin(), gEntries.end());
    return gMatrix;
}

}