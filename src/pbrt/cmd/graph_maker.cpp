#include <pbrt/scene.h>
#include <pbrt/graph/graph.h>
#include <pbrt/graph/voxels/voxel_transmittance.h>
#include <pbrt/util/args.h>

#include <fstream>
#include <regex>

#include "pbrt/graph/lighting_calculator.h"
#include "pbrt/graph/deps/json.hpp"
#include "pbrt/graph/free/free_graph_builder.h"
#include "pbrt/graph/free/free_lighting_calculator.h"
#include "pbrt/graph/free/subdivider.h"

using namespace pbrt;

static void usage() {
    fprintf(stderr,
            R"(
Usage: graph_maker [<options>] <filename.pbrt>

Graph making options:
--config <filename>     Specifies a config file not in the default location
)");
}

void main(int argc, char* argv[]) {
    static_assert(sizeof(double) * CHAR_BIT == 64, "Double must be 64 bits");

    std::vector<std::string> args = GetCommandLineArguments(argv);

    std::vector<std::string> fileNames;
    pstd::optional<std::string> configName;

    for (auto iter = args.begin(); iter != args.end(); ++iter) {
        if ((*iter)[0] != '-') {
            fileNames.push_back(*iter);
            continue;
        }

        auto onError = [](const std::string& error) {
            usage();
            exit(1);
        };

        if (ParseArg(&iter, args.end(), "config", &configName, onError)) {
            // argument parsed
        }
        else if (*iter == "--help" || *iter == "-help" || *iter == "-h") {
            usage();
            exit(0);
        }
        else {
            usage();
            exit(1);
        }
    }

    if (fileNames.empty())
        ErrorExit("Must specify exactly one filename");

    if (!configName) {
        configName = std::regex_replace(fileNames[0], std::regex("\\.pbrt"), ".json");
    }
    std::ifstream configStream(configName.value());

    nlohmann::json jsonConfig;
    try {
        jsonConfig = nlohmann::json::parse(configStream);
    }
    catch (const std::exception& exception) {
        ErrorExit(exception.what());
    }

    auto config = jsonConfig.get<graph::Config>();

    int maxDimensionSteps = std::max(config.graphBuilder.dimensionSteps,
                                     config.subdivider.graphBuilder.dimensionSteps);
    int maxIterationsPerStep = std::max(config.graphBuilder.iterationsPerStep,
                                        config.subdivider.graphBuilder.iterationsPerStep);
    int maxMaxDepth = std::max(config.graphBuilder.maxDepth,
                               config.subdivider.graphBuilder.maxDepth);
    int maxIterations = std::max(
        std::max(config.graphBuilder.edgeTransmittanceIterations, config.lightingCalculator.lightRayIterations),
        std::max(config.subdivider.graphBuilder.edgeTransmittanceIterations, config.subdivider.lightingCalculator.lightRayIterations));

    int maxDiskPoints = util::GetDiskPointsSize(std::max(
        config.lightingCalculator.pointsOnRadius,
        config.subdivider.lightingCalculator.pointsOnRadius));

    int maxSpherePoints = util::GetSphereVolumePointsSize(std::max(
        config.graphBuilder.pointsOnRadius,
        config.subdivider.graphBuilder.pointsOnRadius));

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
    InitPBRT(options);

    BasicScene scene;
    BasicSceneBuilder builder(&scene);
    ParseFiles(&builder, fileNames);

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
    Vector3f lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));

    std::cout << "Vertex radius: " << GetSameSpotRadius(mediumData) * config.graphBuilder.radiusModifier << std::endl;

    Sampler sampler = scene.GetSampler();

    graph::FreeGraphBuilder graphBuilder(mediumData, lightDir, sampler, config.graphBuilder, false);
    graph::FreeGraph graph = graphBuilder.TracePaths();
    std::cout << StringPrintf("Vertices: %s, Edges %s", graph.GetVertices().size(), graph.GetEdges().size()) << std::endl;
    graphBuilder.ComputeTransmittance(graph);

    graph::FreeLightingCalculator lighting(graph, mediumData, lightDir, sampler, config.lightingCalculator, false);
    graph::SparseVec lightVec = lighting.GetLightVector();

    // graph::Subdivider subdivider(graph, mediumData, lightDir, sampler, config.subdivider);
    // subdivider.ComputeSubdivisionEffect(lightVec);

    for (int bouncesIndex = 0; bouncesIndex < config.lightingCalculator.bounces.size(); ++ bouncesIndex) {
        int bounces = config.lightingCalculator.bounces[bouncesIndex];
        int depth = bounces + 1;
        std::cout << StringPrintf("-----------\nDepth %s (%s bounces)", depth, bounces) << std::endl;

        lighting.ComputeFinalLight(lightVec, bouncesIndex);

        float averageLight = 0;
        for (auto& [id, v] : graph.GetVertices())
            averageLight += v.data.lightScalar;
        averageLight /= static_cast<float>(graph.GetVertices().size());
        std::cout << "Average light: " << averageLight << std::endl;

        std::string graphFileName = std::regex_replace(configName.value(),
            std::regex("\\.json"), StringPrintf("_d%s.txt", depth));
        graph.WriteToDisk(graphFileName, graph::basic,
                          graph::StreamFlags{false, false, false, true},
                          graph::StreamOptions{true, false, false});
    }

    CleanupPBRT();
    exit(0);
}
