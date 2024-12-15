#pragma once

#include "voxel_lighting_calculator.h"

#include <iostream>

#include "pbrt/lights.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

VoxelLightingCalculator::VoxelLightingCalculator(Graph& graph, const util::MediumData& mediumData, Vector3f inDirection, Sampler sampler,
    LightingCalculatorConfig config, bool quiet, bool runInParallel)
        : LightingCalculator(graph, mediumData, inDirection, std::move(sampler), config, quiet, runInParallel) {

    uniformGraph = dynamic_cast<UniformGraph*>(&graph);
}

SparseVec VoxelLightingCalculator::GetLightVector() {
    int lightRaysPerVoxelDist = 4;

    float L = 1;
    L /= static_cast<float>(config.lightIterations)
       * static_cast<float>(Sqr(lightRaysPerVoxelDist))
       * uniformGraph->GetSpacing();

    const util::PrimitiveData& primitiveData = mediumData.primitiveData;
    Point3f origin(primitiveData.boundsCenter - inDirection * primitiveData.maxDistToCenter * 2);

    Vector3f xVector;
    Vector3f yVector;
    CoordinateSystem(inDirection, &xVector, &yVector);

    float distBetweenRays = uniformGraph->GetSpacing() / static_cast<float>(lightRaysPerVoxelDist);
    xVector *= distBetweenRays;
    yVector *= distBetweenRays;

    int numSteps = std::ceil(primitiveData.maxDistToCenter / distBetweenRays);

    std::unordered_map<int, float> lightMap;
    lightMap.reserve(numVertices); // rarely need this amount, but better to have enough capacity

    int raysHitting = 0;
    for (int x = -numSteps; x <= numSteps; ++x) {
        for (int y = -numSteps; y <= numSteps; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential gridRay(newOrigin, inDirection);

            if (auto shapeIsect = primitiveData.primitive.Intersect(gridRay, Infinity))
                ++raysHitting;
        }
    }

    int workNeeded = raysHitting * config.lightIterations;
    ProgressReporter progress(workNeeded, "Computing initial lighting", quiet);

    int numRaysScatteredOutsideGrid = 0;

    for (int x = -numSteps; x <= numSteps; ++x) {
        for (int y = -numSteps; y <= numSteps; ++y) {
            Point3f newOrigin = origin + xVector * x + yVector * y;
            RayDifferential gridRay(newOrigin, inDirection);

            auto shapeIsect = primitiveData.primitive.Intersect(gridRay, Infinity);
            if (!shapeIsect)
                continue;

            shapeIsect->intr.SkipIntersection(&gridRay, shapeIsect->tHit);

            if (!gridRay.medium) {
                continue;
            }

            shapeIsect = primitiveData.primitive.Intersect(gridRay, Infinity);
            if (!shapeIsect)
                continue;

            float tMax = shapeIsect->tHit;

            for (int i = 0; i < config.lightIterations; ++i) {
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
                            coors = uniformGraph->FitToGraph(p).first;
                            return false;
                        }
                        else {
                            // Handle null scattering along ray path
                            return true;
                        }
                    });

                if (coors) {
                    OptRefConst<Vertex> optVertex = uniformGraph->GetVertexConst(coors.value());
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

    return LightMapToVector(lightMap);
}

SparseMat VoxelLightingCalculator::GetGMatrix() const {
    auto voxelSize = static_cast<float>(std::pow(uniformGraph->GetSpacing(), 3));

    SparseMat gMatrix = LightingCalculator::GetGMatrix();
    gMatrix *= voxelSize;

    return gMatrix;
}

}