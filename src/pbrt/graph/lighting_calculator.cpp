#include "lighting_calculator.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/media.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

LightingCalculator::LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                                       DistantLight* light, Sampler sampler, const std::vector<RefConst<Vertex>>& litVertices)
    : grid(grid), mediumData(mediumData), light(light), sampler(std::move(sampler)), litVertices(litVertices) {

    if (!mediumData.medium.Is<HomogeneousMedium>())
        ErrorExit("Only homogeneous media supported");

    if (!HasSequentialIds())
        ErrorExit("Grid must have sequential ids for mapping to matrices");

    numVertices = static_cast<int>(grid.GetVertices().size());

    lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));
}

UniformGraph LightingCalculator::GetFinalLightGrid(int initialLightingIterations, int transmittanceIterations) {
    SparseVec light = GetLightVector(initialLightingIterations);

    if (transmittanceIterations > 0) {
        SparseMat transmittance = GetTransmittanceMatrix();
        SparseVec curLight = light;

        ProgressReporter progress(transmittanceIterations - 1, "Computing final lighting", false);

        for (int i = 0; i < transmittanceIterations; ++i) {
            curLight = transmittance * curLight;
            light += curLight;
            progress.Update();
        }
        progress.Done();
    }

    UniformGraph finalGrid(grid.GetSpacing());

    for (auto& pair : grid.GetVertices()) {
        const Vertex& v = pair.second;
        finalGrid.AddVertex(v.id, v.coors.value(), VertexData{{}, light.coeffRef(v.id)});
    }

    return finalGrid;
}

bool LightingCalculator::HasSequentialIds() const {
    int total = 0;
    int max = 0;

    for (auto& pair : grid.GetVertices()) {
        total += pair.first;
        max = std::max(max, pair.first);
    }

    int oddInMiddle = max % 2 == 1 ? (max / 2 + 1) : 0;
    int totalExpected = (max + 1) * (max / 2) + oddInMiddle;

    return total == totalExpected;
}

SparseVec LightingCalculator::GetLightVector(int initialLightingIterations) {
    // lambda only relevant parameter for distant light
    SampledSpectrum L = light->SampleLi(LightSampleContext{Interaction()},
                                        Point2f(), mediumData.lambda, false).value().L;

    ProgressReporter progress(static_cast<int>(litVertices.size()) * initialLightingIterations,
        "Computing initial lighting", false);

    float maxRayLength = mediumData.maxDistToCenter * 2;

    std::unordered_map<int, SampledSpectrum> lightMap;
    lightMap.reserve(numVertices); // rarely need this amount, but better to have enough capacity

    for (auto vertex : litVertices) {
        for (int i = 0; i < initialLightingIterations; ++i) {
            // Initialize _RNG_ for sampling the majorant transmittance
            uint64_t hash0 = Hash(sampler.Get1D());
            uint64_t hash1 = Hash(sampler.Get1D());
            RNG rng(hash0, hash1);

            RayDifferential ray(vertex.get().point - lightDir * maxRayLength, lightDir);

            auto shapeInter = mediumData.aggregate->Intersect(ray, Infinity);
            shapeInter->intr.SkipIntersection(&ray, shapeInter->tHit);
            float tMax = mediumData.aggregate->Intersect(ray, Infinity).value().tHit;

            std::optional<Point3i> coors;

            // Sample new point on ray
            SampleT_maj(
                (Ray&)ray, tMax, sampler.Get1D(), rng, mediumData.lambda,
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
                        coors = grid.FitToGraph(p).first;
                        return false;
                    }
                    else {
                        // Handle null scattering along ray path
                        return true;
                    }
                });

            if (coors) {
                int id = grid.GetVertexConst(coors.value()).value().get().id;
                if (lightMap.find(id) == lightMap.end())
                    lightMap[id] = SampledSpectrum(0);

                lightMap[id] += L / static_cast<float>(initialLightingIterations);
            }

            progress.Update();
        }
    }

    progress.Done();

    std::vector<std::pair<int, SampledSpectrum>> lightPairs(lightMap.begin(), lightMap.end());
    std::sort(lightPairs.begin(), lightPairs.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    SparseVec lightVector(numVertices);
    for (auto& pair : lightPairs) {
        lightVector.coeffRef(pair.first) = pair.second;
    }

    return lightVector;
}

SparseMat LightingCalculator::GetPhaseMatrix() const {
    std::vector<Eigen::Triplet<SampledSpectrum>> phaseEntries;
    phaseEntries.reserve(numVertices);

    for (int i = 0; i < numVertices; ++i)
        phaseEntries.emplace_back(i, i, SampledSpectrum(1 / (4 * Pi)));

    SparseMat phaseMatrix(numVertices, numVertices);
    phaseMatrix.setFromTriplets(phaseEntries.begin(), phaseEntries.end());
    return phaseMatrix;
}

SparseMat LightingCalculator::GetGMatrix() const {
    auto& vertices = grid.GetVertices();
    auto& edges = grid.GetEdges();

    std::vector<Eigen::Triplet<SampledSpectrum>> gEntries;
    gEntries.reserve(grid.GetEdges().size());

    for (auto& edge : edges) {
        const Vertex& from = vertices.at(edge.second.from);
        const Vertex& to = vertices.at(edge.second.to);

        SampledSpectrum T = edge.second.data.throughput;
        double voxelSize = std::pow(grid.GetSpacing(), 3);
        double edgeLength = std::pow(Length(from.point - to.point), 2);
        auto G = static_cast<float>(voxelSize / edgeLength);

        gEntries.emplace_back(to.id, from.id, T * G);
    }

    SparseMat gMatrix(numVertices, numVertices);
    gMatrix.setFromTriplets(gEntries.begin(), gEntries.end());
    return gMatrix;
}

SparseMat LightingCalculator::GetTransmittanceMatrix() const {
    return GetPhaseMatrix() * GetGMatrix() / SampledSpectrum(static_cast<float>(numVertices));
}

}