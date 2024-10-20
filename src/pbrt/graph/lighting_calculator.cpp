#include "lighting_calculator.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

LightingCalculator::LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                                       DistantLight* light, Sampler sampler)
    : transmittanceGrid(grid), mediumData(mediumData), light(light), sampler(std::move(sampler)) {

    if (!HasSequentialIds())
        ErrorExit("Grid must have sequential ids for mapping to matrices");

    numVertices = static_cast<int>(grid.GetVerticesConst().size());

    lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));
}

UniformGraph LightingCalculator::GetFinalLightGrid(int initialLightingIterations, int lightRaysPerVoxelDist, int transmittanceIterations) {
    SparseVec light = GetLightVector(initialLightingIterations, lightRaysPerVoxelDist);

    if (transmittanceIterations > 0) {
        SparseMat transmittance = GetTransmittanceMatrix();
        SparseVec curLight = light;

        ProgressReporter progress(transmittanceIterations, "Computing final lighting", false);

        for (int i = 0; i < transmittanceIterations; ++i) {
            curLight = transmittance * curLight;
            light += curLight;
            progress.Update();
        }
        progress.Done();
    }

    UniformGraph finalGrid(transmittanceGrid.GetSpacing());

    for (auto& pair : transmittanceGrid.GetVerticesConst()) {
        const Vertex& v = pair.second;
        finalGrid.AddVertex(v.id, v.coors.value(), VertexData{{}, light.coeffRef(v.id)});
    }

    return finalGrid;
}

bool LightingCalculator::HasSequentialIds() const {
    int total = 0;
    int max = 0;

    for (auto& pair : transmittanceGrid.GetVerticesConst()) {
        total += pair.first;
        max = std::max(max, pair.first);
    }

    int oddInMiddle = max % 2 == 1 ? (max / 2 + 1) : 0;
    int totalExpected = (max + 1) * (max / 2) + oddInMiddle;

    return total == totalExpected;
}

SparseVec LightingCalculator::GetLightVector(int initialLightingIterations, int lightRaysPerVoxelDist) {
    if (initialLightingIterations <= 0)
        ErrorExit("Must have at least one initial lighting iteration");

    if (lightRaysPerVoxelDist <= 0)
        ErrorExit("Must have at least one ray per voxel");

    float L = 1;
    L /= static_cast<float>(initialLightingIterations)
       * static_cast<float>(std::pow(lightRaysPerVoxelDist, 2))
       * transmittanceGrid.GetSpacing();

    Point3f origin(mediumData.boundsCenter - lightDir * mediumData.maxDistToCenter * 2);

    Vector3f xVector;
    Vector3f yVector;
    CoordinateSystem(lightDir, &xVector, &yVector);

    float distBetweenRays = transmittanceGrid.GetSpacing() / static_cast<float>(lightRaysPerVoxelDist);
    xVector *= distBetweenRays;
    yVector *= distBetweenRays;

    int numSteps = std::ceil(mediumData.maxDistToCenter / distBetweenRays);

    std::unordered_map<int, float> lightMap;
    lightMap.reserve(numVertices); // rarely need this amount, but better to have enough capacity

    int raysHitting = 0;
    for (int x = -numSteps; x <= numSteps; ++x) {
        for (int y = -numSteps; y <= numSteps; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential gridRay(newOrigin, lightDir);

            if (auto shapeIsect = mediumData.aggregate->Intersect(gridRay, Infinity))
                ++raysHitting;
        }
    }

    int workNeeded = raysHitting * initialLightingIterations;
    ProgressReporter progress(workNeeded, "Computing initial lighting", false);

    numRaysScatteredOutsideGrid = 0;

    for (int x = -numSteps; x <= numSteps; ++x) {
        for (int y = -numSteps; y <= numSteps; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential gridRay(newOrigin, lightDir);

            auto shapeIsect = mediumData.aggregate->Intersect(gridRay, Infinity);
            if (!shapeIsect)
                continue;

            shapeIsect->intr.SkipIntersection(&gridRay, shapeIsect->tHit);

            if (!gridRay.medium) {
                continue;
            }

            shapeIsect = mediumData.aggregate->Intersect(gridRay, Infinity);
            if (!shapeIsect)
                continue;

            float tMax = shapeIsect->tHit;

            for (int i = 0; i < initialLightingIterations; ++i) {
                sampler.StartPixelSample(Point2i(x, y), i);

                // Initialize _RNG_ for sampling the majorant transmittance
                uint64_t hash0 = Hash(sampler.Get1D());
                uint64_t hash1 = Hash(sampler.Get1D());
                RNG rng(hash0, hash1);

                std::optional<Point3i> coors;

                // Sample new point on ray
                SampleT_maj(
                    (Ray&)gridRay, tMax, sampler.Get1D(), rng, mediumData.defaultLambda,
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
                            // Stop path sampling if maximum depth has been reached
                            coors = transmittanceGrid.FitToGraph(p).first;
                            return false;
                        }
                        else {
                            // Handle null scattering along ray path
                            return true;
                        }
                    });

                if (coors) {
                    OptRefConst<Vertex> optVertex = transmittanceGrid.GetVertexConst(coors.value());
                    if (!optVertex) {
                        ++numRaysScatteredOutsideGrid;
                        progress.Update();
                        continue;
                    }

                    int id = optVertex->get().id;
                    if (lightMap.find(id) == lightMap.end())
                        lightMap[id] = 0;

                    lightMap[id] += L;
                }

                progress.Update();
            }
        }
    }

    progress.Done();

    std::cout << numRaysScatteredOutsideGrid << " / " << workNeeded << " rays scattered outsided grid" << std::endl;

    std::vector<std::pair<int, float>> lightPairs(lightMap.begin(), lightMap.end());
    std::sort(lightPairs.begin(), lightPairs.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    SparseVec lightVector(numVertices);
    for (auto& pair : lightPairs) {
        lightVector.coeffRef(pair.first) = pair.second;
    }

    return lightVector;
}

SparseMat LightingCalculator::GetPhaseMatrix() const {
    std::vector<Eigen::Triplet<float>> phaseEntries;
    phaseEntries.reserve(numVertices);

    for (int i = 0; i < numVertices; ++i)
        phaseEntries.emplace_back(i, i, Inv4Pi);

    SparseMat phaseMatrix(numVertices, numVertices);
    phaseMatrix.setFromTriplets(phaseEntries.begin(), phaseEntries.end());
    return phaseMatrix;
}

SparseMat LightingCalculator::GetGMatrix() const {
    auto& vertices = transmittanceGrid.GetVerticesConst();
    auto& edges = transmittanceGrid.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> gEntries;
    gEntries.reserve(transmittanceGrid.GetEdgesConst().size());

    for (auto& edge : edges) {
        const Vertex& from = vertices.at(edge.second.from);
        const Vertex& to = vertices.at(edge.second.to);

        float T = edge.second.data.throughput;
        double voxelSize = std::pow(transmittanceGrid.GetSpacing(), 3);
        double edgeLength = std::pow(Length(from.point - to.point), 2);
        auto G = static_cast<float>(voxelSize / edgeLength);

        gEntries.emplace_back(to.id, from.id, T * G);
    }

    SparseMat gMatrix(numVertices, numVertices);
    gMatrix.setFromTriplets(gEntries.begin(), gEntries.end());
    return gMatrix;
}

SparseMat LightingCalculator::GetTransmittanceMatrix() const {
    return GetPhaseMatrix() * GetGMatrix();
}

}