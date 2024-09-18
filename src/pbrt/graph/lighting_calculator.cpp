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
    if (transmittanceIterations < 0  || transmittanceIterations > 1)
        ErrorExit("Must be zero or one iteration");

    auto light = GetLightVector(initialLightingIterations);

    for (int i = 0; i < NSpectrumSamples; ++i) {
        if (transmittanceIterations == 1)
            light += GetTransmittanceMatrix() * light;
    }

    UniformGraph finalGrid(grid.GetSpacing());

    for (auto& pair : grid.GetVertices()) {
        auto& v = pair.second;
        finalGrid.AddVertex(v.id, v.coors.value(), VertexData{{}, light[v.id]});
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

Matrix<SampledSpectrum, Dynamic, 1> LightingCalculator::GetLightVector(int initialLightingIterations) {
    // lambda only relevant parameter for distant light
    SampledSpectrum L = light->SampleLi(LightSampleContext{Interaction()},
                                        Point2f(), mediumData.lambda, false).value().L;

    ProgressReporter progress(static_cast<int>(litVertices.size()) * initialLightingIterations,
        "Computing initial lighting", false);

    float maxRayLength = mediumData.maxDistToCenter * 2;
    Matrix<SampledSpectrum, Dynamic, 1> lightVector = Matrix<SampledSpectrum, Dynamic, 1>::Zero(numVertices);

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
                int scatterId = grid.GetVertexConst(coors.value()).value().get().id;
                lightVector(scatterId) += L / static_cast<float>(initialLightingIterations);
            }

            progress.Update();
        }
    }

    progress.Done();
    return lightVector;
}

Matrix<SampledSpectrum, Dynamic, Dynamic> LightingCalculator::GetPhaseMatrix() const {
    Matrix<SampledSpectrum, Dynamic, Dynamic> phase = Matrix<SampledSpectrum, Dynamic, Dynamic>::Zero(numVertices, numVertices);

    for (int i = 0; i < numVertices; ++i)
        phase(i, i) = SampledSpectrum(1 / (4 * Pi));

    return phase;
}

Matrix<SampledSpectrum, Dynamic, Dynamic> LightingCalculator::GetGMatrix() const {
    auto& vertices = grid.GetVertices();
    auto& edges = grid.GetEdges();
    Matrix<SampledSpectrum, Dynamic, Dynamic> gMatrix = Matrix<SampledSpectrum, Dynamic, Dynamic>::Zero(numVertices, numVertices);

    for (auto& vertexPair : grid.GetVertices()) {
        for (auto& edgePair : vertexPair.second.outEdges) {
            const Vertex& from = vertexPair.second;
            const Vertex& to = vertices.at(edgePair.first);
            const Edge& edge = edges.at(edgePair.second);

            SampledSpectrum T = edge.data.throughput;
            double voxelSize = std::pow(grid.GetSpacing(), 3);
            double edgeLength = std::pow(Length(from.point - to.point), 2);
            auto G = static_cast<float>(voxelSize / edgeLength);
            gMatrix(to.id, from.id) = T * G;
        }
    }

    return gMatrix;
}

Matrix<SampledSpectrum, Dynamic, Dynamic> LightingCalculator::GetTransmittanceMatrix() const {
    return GetPhaseMatrix() * GetGMatrix() / SampledSpectrum(static_cast<float>(numVertices));
}

}