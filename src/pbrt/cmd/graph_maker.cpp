#include <graph/graph.h>
#include <pbrt/scene.h>
#include <pbrt/util/args.h>

#include <fstream>
#include <regex>

#include "graph/lighting_calculator.h"
#include "graph/deps/json.hpp"
#include "graph/free/free_graph_builder.h"
#include "pbrt/lights.h"

using namespace pbrt;

static void usage() {
    fprintf(stderr,
            R"(
Usage: graph_maker [<options>] <filename.pbrt>

Graph making options:
--config <filename>         Specifies a config file not in the default location
--node-radius <radius>      Absolute node radius, overrides the node radius specified in the config file
--quiet                     If flag is present, no output is written to std::cout
)");
}

int main(int argc, char* argv[]) {
    static_assert(sizeof(double) * CHAR_BIT == 64, "Double must be 64 bits");

    std::vector<std::string> args = GetCommandLineArguments(argv);

    std::vector<std::string> fileNames;
    pstd::optional<std::string> configName;
    pstd::optional<float> nodeRadius;
    bool quiet = false;

    for (auto iter = args.begin(); iter != args.end(); ++iter) {
        if ((*iter)[0] != '-') {
            fileNames.push_back(*iter);
            continue;
        }

        auto onError = [](const std::string& error) {
            usage();
            exit(1);
        };

        if (ParseArg(&iter, args.end(), "config", &configName, onError) ||
            ParseArg(&iter, args.end(), "node-radius", &nodeRadius, onError) ||
            ParseArg(&iter, args.end(), "quiet", &quiet, onError)) {
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

    int dimensionSteps = config.graphBuilder.dimensionSteps;
    int iterationsPerStep = config.graphBuilder.iterationsPerStep;
    int maxDepth = config.graphBuilder.maxDepth;

    int maxIterations = config.lightingCalculator.lightIterations;

    int maxDiskPoints = util::GetDiskPointsSize(config.lightingCalculator.pointsOnRadiusLight);
    int maxSpherePoints = std::max(config.graphBuilder.edgeReinforcement.reinforcementRays,
                                    config.graphBuilder.neighbourReinforcement.reinforcementRays);
    int maxRaysPerVertex = std::max(maxDiskPoints, maxSpherePoints);

    int numRays = Sqr(dimensionSteps);
    int64_t maxNumVertices = static_cast<int64_t>(numRays) * iterationsPerStep * maxDepth;
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
    Sampler sampler = scene.GetSampler();

    util::MediumData mediumData(accel, scene.GetCamera().GetFilm().SampleWavelengths(0));
    light->Preprocess(mediumData.primitiveData.bounds);
    Vector3f lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));

    auto processStart = std::chrono::steady_clock::now();

    if (nodeRadius.has_value())
        Warning("Use of the 'node-radius' flag is only intended for development purposes");
    graph::FreeGraphBuilder graphBuilder(mediumData, lightDir, sampler, config.graphBuilder, quiet, nodeRadius);
    graph::FreeGraph graph = graphBuilder.BuildGraph();

    auto writeStats = [&]() -> void {
        auto processEnd = std::chrono::steady_clock::now();
        int processDuration = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(processEnd - processStart).count());

        std::string statsFileName = std::regex_replace(configName.value(),
                std::regex("\\.json"), "_stats.json");
        std::string fullPath = util::FileNameToPath(statsFileName);
        util::CreateParentDirectories(fullPath);

        graph::json stats;
        graph.AddStats(stats);
        stats["duration"] = {{"seconds", processDuration}, {"formatted", util::FormatTime(processDuration)}};
        stats["sizes"] = {{"node_radius", graph.GetVertexRadius()},
                             {"scene_radius", mediumData.primitiveData.maxDistToCenter}};

        if (!quiet) {
            std::cout << "=== Graph stats ===" << std::endl;
            std::cout << std::setw(2) << stats << std::endl;
            std::cout << "===================" << std::endl;
        }

        std::ofstream file(fullPath);
        file << std::setw(4) << stats << std::endl;
        file.close();
    };

    if (config.lightingCalculator.bounces.size() > 1)
        writeStats();

    graph::LightingCalculator lighting(graph, mediumData, lightDir, sampler, config.lightingCalculator, quiet);
    graph::SparseVec lightVec = lighting.GetLightVector();
    graph::SparseMat transportMat = lighting.GetTransportMatrix();

    graph.streamOptions = graph::StreamOptions{true, false, false};
    graph.streamFlags = graph::StreamFlags{false, false, false, true, true};

    for (int bouncesIndex = 0; bouncesIndex < config.lightingCalculator.bounces.size(); ++bouncesIndex) {
        int bounces = config.lightingCalculator.bounces[bouncesIndex];
        int depth = bounces + 1;
        std::cout << StringPrintf("-----------\nDepth %s (%s bounces)", depth, bounces) << std::endl;

        int bouncesComputed = lighting.ComputeFinalLight(lightVec, transportMat, bouncesIndex);
        int depthComputed = bouncesComputed + 1;

        if (config.lightingCalculator.bounces.size() == 1)
            writeStats();

        if (!quiet) {
            util::Averager lightAverager(static_cast<int>(graph.GetVertices().size()));
            for (auto& [id, v] : graph.GetVertices())
                lightAverager.AddValue(v.data.lightScalar);

            std::cout << StringPrintf("Light: %s", lightAverager.PrintInfo()) << std::endl;
        }

        std::string graphFileName = std::regex_replace(configName.value(),
            std::regex("\\.json"), StringPrintf("_d%s.txt", depthComputed));

        graph.WriteToDisk(graphFileName, graph::basic);


        if (depth != depthComputed) {
            std::cout << StringPrintf("Numerical limits reached with depth %s, depth %s not possible", depthComputed, depth) << std::endl;
            break;
        }
    }

    CleanupPBRT();
    exit(0);
}
