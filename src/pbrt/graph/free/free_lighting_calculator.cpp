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
    int numEntryVertices = 0;
    for (auto& pair : graph.GetVertices()) {
        if (pair.second.data.type == entry)
            ++numEntryVertices;
    }

    std::unordered_map<int, float> lightMap;
    std::vector<int> entryVertexIds;
    lightMap.reserve(numEntryVertices);
    entryVertexIds.reserve(numEntryVertices);

    for (auto& [id, vertex] : graph.GetVertices()) {
        if (vertex.data.type == entry) {
            lightMap[id] = 0; // allocate space for concurrent access
            entryVertexIds.push_back(id);
        }
    }

    ThreadLocal<Sampler> samplers([&] { return sampler.Clone(); });

    float sphereRadius = freeGraph->GetVertexRadius().value();
    util::SphereMaker sphereMaker(sphereRadius);

    int resolutionDimensionSize = Options->graph.samplingResolution->x;

    int64_t workNeeded = static_cast<int64_t>(numEntryVertices) * config.lightRayIterations * util::GetDiskPointsSize(config.pointsOnRadius);
    ProgressReporter progress(workNeeded, "Computing initial lighting", quiet);

    ParallelFor(0, numEntryVertices, config.runInParallel, [&](int listIndex) {
        Sampler& samplerClone = samplers.Get();

        int vertexId = entryVertexIds[listIndex];
        Vertex& vertex = graph.GetVertex(vertexId)->get();

        SphereContainer currentSphere = sphereMaker.GetSphereFor(vertex.point);

        Point3f origin = vertex.point - inDirection * mediumData.primitiveData.maxDistToCenter * 2;
        std::vector<Point3f> diskPoints = util::GetDiskPoints(origin, sphereRadius, config.pointsOnRadius, inDirection);

        util::AverageDenoiser averageDenoiser(static_cast<int>(diskPoints.size()));

        uint64_t startIndex = listIndex * diskPoints.size();
        for (int pointIndex = 0; pointIndex < diskPoints.size(); ++pointIndex) {
            uint64_t curIndex = startIndex + pointIndex;
            int yCoor = static_cast<int>(curIndex / resolutionDimensionSize);
            int xCoor = static_cast<int>(curIndex - yCoor * resolutionDimensionSize);

            Point3f diskPoint = diskPoints[pointIndex];
            RayDifferential ray(diskPoint, inDirection);

            util::HitsResult mediumHits = GetHits(mediumData.primitiveData.primitive, ray, mediumData);
            util::HitsResult sphereHits = GetHits(currentSphere.sphere, ray, mediumData);

            if (mediumHits.type != util::OutsideTwoHits || sphereHits.type != util::OutsideTwoHits) {
                progress.Update(config.lightRayIterations);
                continue;
            }

            util::StartEndT startEnd = GetStartEndT(mediumHits, sphereHits);

            if (startEnd.sphereRayNotInMedium) {
                progress.Update(config.lightRayIterations);
                continue;
            }

            mediumHits.intersections[0].intr.SkipIntersection(&ray, startEnd.startT);

            float rayTotal = 0;
            for (int i = 0; i < config.lightRayIterations; ++i) {
                samplerClone.StartPixelSample({xCoor, yCoor}, i);

                if (SampleScatterNearPoint(ray, vertex.point, sphereRadius, startEnd.endT, samplerClone, mediumData))
                    rayTotal += 1;

                progress.Update();
            }

            float distInSphereNormalizer = 1.f / (startEnd.endScatterT - startEnd.startScatterT);
            averageDenoiser.AddValue(rayTotal * distInSphereNormalizer / static_cast<float>(config.lightRayIterations));
        }
        lightMap[vertexId] = averageDenoiser.GetAverage();
    });
    progress.Done();

    return LightMapToVector(lightMap);
}

SparseMat FreeLightingCalculator::GetGMatrix() const {
    auto& vertices = graph.GetVerticesConst();
    auto& edges = graph.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> gEntries;
    gEntries.reserve(edges.size());

    for (auto& edge : edges) {
        const Vertex& from = vertices.at(edge.second.from);
        const Vertex& to = vertices.at(edge.second.to);

        float T = edge.second.data.throughput;
        // float G = std::max(std::min(1 / LengthSquared(from.point - to.point), 100.f), 1.f);

        gEntries.emplace_back(to.id, from.id, T);
    }

    SparseMat gMatrix(numVertices, numVertices);
    gMatrix.setFromTriplets(gEntries.begin(), gEntries.end());
    return gMatrix;
}

SparseMat FreeLightingCalculator::GetTransmittanceMatrix() const {
    std::vector<Eigen::Triplet<float>> weightEntries;
    weightEntries.reserve(numVertices);

    for (int i = 0; i < numVertices; ++i) {
        int numEdges = static_cast<int>(graph.GetVertex(i)->get().inEdges.size());
        float weight = 1 / static_cast<float>(numEdges);

        if (numEdges > 0)
            weightEntries.emplace_back(i, i, weight);
    }

    SparseMat weightMatrix(numVertices, numVertices);
    weightMatrix.setFromTriplets(weightEntries.begin(), weightEntries.end());

    return weightMatrix * LightingCalculator::GetTransmittanceMatrix();
}

}