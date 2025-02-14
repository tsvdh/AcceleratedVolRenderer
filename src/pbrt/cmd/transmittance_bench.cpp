#include <climits>

#include "pbrt/lights.h"
#include "pbrt/pbrt.h"
#include "pbrt/scene.h"
#include "pbrt/graph/util.h"
#include "pbrt/graph/deps/json.hpp"

using namespace pbrt;

void main(int argc, int* argv[]) {
    static_assert(sizeof(double) * CHAR_BIT == 64, "Double must be 64 bits");

    std::vector distances {0.1f, 0.2f, 0.5f, 1.0f};
    std::vector rayIterations {1, 3, 5, 7, 10, 20, 2048};
    int iterations = 100;

    int maxSampleDimensionSize = std::ceil(std::sqrt(distances.size() * rayIterations.size() * iterations));

    PBRTOptions options;
    options.disablePixelJitter = true;
    options.disableWavelengthJitter = true;
    options.renderingSpace = RenderingCoordinateSystem::World;
    options.pixelSamples = rayIterations[rayIterations.size() - 1];
    options.graph.samplingResolution = Point2i(maxSampleDimensionSize, maxSampleDimensionSize);

    InitPBRT(options);

    BasicScene scene;
    BasicSceneBuilder builder(&scene);
    ParseFiles(&builder, {"C:/Users/tsvdh/CodeProjects/VSCode/PbrtScenes/cube/cube.pbrt"});

    std::map<std::string, Medium> media = scene.CreateMedia();

    NamedTextures textures = scene.CreateTextures();
    std::map<int, pstd::vector<Light>*> shapeIndexToAreaLights;
    std::vector<Light> lights = scene.CreateLights(textures, &shapeIndexToAreaLights);
    DistantLight* light = util::GetLight(lights);

    std::map<std::string, Material> namedMaterials;
    std::vector<Material> materials;
    scene.CreateMaterials(textures, &namedMaterials, &materials);
    Primitive accel = scene.CreateAggregate(textures, shapeIndexToAreaLights, media, namedMaterials, materials);

    util::MediumData mediumData(accel, scene.GetCamera().GetFilm().SampleWavelengths(0));
    light->Preprocess(mediumData.primitiveData.bounds);

    Sampler sampler = scene.GetSampler();
    ScratchBuffer buffer;

    for (int a = 0; a < distances.size(); ++a) {
        RayDifferential ray({distances[a], 0, 0}, {-1, 0, 0}, 0, mediumData.medium);
        util::StartEndT startEnd{0, distances[a] + 1, distances[a], distances[a] + 1, false};

        uint64_t bBaseIndex = a * rayIterations.size();
        for (int b = 0; b < rayIterations.size(); ++b) {
            uint64_t bCurIndex = bBaseIndex + b;

            util::Averager averager(iterations);

            uint64_t iterationsBaseIndex = bCurIndex * iterations;
            for (int i = 0; i < iterations; ++i) {
                uint64_t iterationsCurIndex = iterationsBaseIndex + i;

                averager.AddValue(graph::ComputeRaysScatteredInSphere(ray, startEnd, mediumData, sampler, buffer,
                    rayIterations[b], iterationsCurIndex));
            }

            auto [average, std, variance] = averager.GetInfo();
            std::cout << StringPrintf("(%s %s), %s %s", distances[a], rayIterations[b], average, std) << std::endl;
        }
    }
}