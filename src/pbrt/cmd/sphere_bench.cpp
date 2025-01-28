#include <fstream>
#include <limits.h>

#include "pbrt/lights.h"
#include "pbrt/pbrt.h"
#include "pbrt/scene.h"
#include "pbrt/graph/graph.h"
#include "pbrt/graph/free/free_graph_builder.h"
#include "pbrt/graph/free/free_lighting_calculator.h"

using namespace pbrt;

struct LightReport {
    float averageLight;
    float duration;
    float graphRadius;
};

LightReport reportLight(graph::Config config) {
    int maxDimensionSteps = config.graphBuilder.dimensionSteps;
    int maxIterationsPerStep = config.graphBuilder.iterationsPerStep;
    int maxMaxDepth = config.graphBuilder.maxDepth;
    int maxIterations = std::max(config.graphBuilder.edgeTransmittanceIterations, config.lightingCalculator.lightRayIterations);

    int maxDiskPoints = util::GetDiskPointsSize(config.lightingCalculator.pointsOnRadius);
    int maxSpherePoints = util::GetSphereVolumePointsSize(config.graphBuilder.pointsOnRadius);
    int maxRaysPerVertex = std::max(maxDiskPoints, maxSpherePoints);

    int64_t numRays = Sqr(maxDimensionSteps);
    int64_t maxNumVertices = numRays * maxIterationsPerStep * maxMaxDepth;
    int64_t maxNumRays = maxNumVertices * maxRaysPerVertex;
    int64_t maxSampleDimensionSizeAsLong = std::ceil(std::sqrt(maxNumRays));
    int pixelSamples = RoundUpPow2(maxIterations);

    if (maxSampleDimensionSizeAsLong > std::numeric_limits<int>::max())
        ErrorExit("Dimension size too big");
    int maxSampleDimensionSizeAsInt = static_cast<int>(maxSampleDimensionSizeAsLong);

    PBRTOptions options;
    options.disablePixelJitter = true;
    options.disableWavelengthJitter = true;
    options.renderingSpace = RenderingCoordinateSystem::World;
    options.pixelSamples = pixelSamples;
    options.graph.samplingResolution = Point2i(maxSampleDimensionSizeAsInt, maxSampleDimensionSizeAsInt);

    delete Options;
    Options = new PBRTOptions(options);

    BasicScene scene;
    BasicSceneBuilder builder(&scene);
    ParseFiles(&builder, {"C:/Users/tsvdh/CodeProjects/VSCode/PbrtScenes/sphere/sphere.pbrt"});

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

    Vector3f lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));

    int iterations = 20;
    float totalAverageLight = 0;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        graph::FreeGraphBuilder graphBuilder(mediumData, lightDir, sampler, config.graphBuilder, true, i);
        graph::FreeGraph graph = graphBuilder.TracePaths();
        graphBuilder.ComputeTransmittance(graph);

        graph::FreeLightingCalculator lighting(graph, mediumData, lightDir, sampler, config.lightingCalculator, true);
        lighting.ComputeFinalLight();

        float averageLight = 0;
        for (auto& [id, v] : graph.GetVertices()) {
            averageLight += v.data.lightScalar;
        }
        totalAverageLight += averageLight / graph.GetVertices().size();
    }

    totalAverageLight /= iterations;

    auto end = std::chrono::high_resolution_clock::now();
    float totalDuration = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) / iterations;

    float mediumRadius = mediumData.primitiveData.bounds.pMax.x;
    float graphRadius = GetSameSpotRadius(mediumData) * config.graphBuilder.radiusModifier;
    float radiusRatio = graphRadius / mediumRadius;

    return {totalAverageLight, totalDuration, radiusRatio};
}

void main(int argc, char* argv[]) {
    static_assert(sizeof(double) * CHAR_BIT == 64, "Double must be 64 bits");

    std::ifstream configStream("C:/Users/tsvdh/CodeProjects/VSCode/PbrtScenes/sphere/sphere.json");

    nlohmann::json jsonConfig;
    try {
        jsonConfig = nlohmann::json::parse(configStream);
    }
    catch (const std::exception& exception) {
        ErrorExit(exception.what());
    }

    auto config = jsonConfig.get<graph::Config>();

    InitPBRT(PBRTOptions());

    std::vector dimensionSteps = {5, 10};
    std::vector iterationsPerStep = {50, 100, 200};
    std::vector radiusModifier = {50, 75, 100, 125, 150};

    for (int a = 0; a < dimensionSteps.size(); a++) {
        for (int b = 0; b < iterationsPerStep.size(); b++) {
            for (int c = 0; c < radiusModifier.size(); ++c) {
                config.graphBuilder.dimensionSteps = dimensionSteps[a];
                config.graphBuilder.iterationsPerStep = iterationsPerStep[b];
                config.graphBuilder.radiusModifier = radiusModifier[c];

                LightReport report = reportLight(config);
                std::cout << "(" << dimensionSteps[a] << " " << iterationsPerStep[b] << " " << radiusModifier[c] << " " << report.graphRadius << ") "
                    << report.averageLight << " in " << report.duration << "ms" << std::endl;
            }
        }
    }
}