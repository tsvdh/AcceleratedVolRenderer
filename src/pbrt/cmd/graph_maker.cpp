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
        } else if (*iter == "--help" || *iter == "-help" || *iter == "-h") {
            usage();
            exit(0);
        } else {
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
    } catch (const std::exception& exception) {
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
        std::max(config.graphBuilder.edgeIterations, config.lightingCalculator.lightIterations),
        std::max(config.subdivider.graphBuilder.edgeIterations, config.subdivider.lightingCalculator.lightIterations));

    int numRays = Sqr(maxDimensionSteps);
    int maxNumVertices = numRays * maxIterationsPerStep * maxMaxDepth;
    int maxSampleDimensionSize = std::ceil(std::sqrt(maxNumVertices));
    int pixelSamples = RoundUpPow2(maxIterations);

    PBRTOptions options;
    options.disablePixelJitter = true;
    options.disableWavelengthJitter = true;
    options.renderingSpace = RenderingCoordinateSystem::World;
    options.pixelSamples = pixelSamples;
    options.graph.samplingResolution = Point2i(maxSampleDimensionSize, maxSampleDimensionSize);
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

    Sampler sampler = scene.GetSampler();

    Vector3f lightDir = -Normalize(light->GetRenderFromLight()(Vector3f(0, 0, 1)));

    graph::FreeGraphBuilder graphBuilder(mediumData, lightDir, sampler, config.graphBuilder, false, true);
    graph::FreeGraph graph = graphBuilder.TracePaths();
    graphBuilder.ComputeTransmittance(graph);

    graph::FreeLightingCalculator lighting(graph, mediumData, lightDir, sampler, config.lightingCalculator, false, true);
    graph::SparseVec lightVec = lighting.GetLightVector();

    graph::Subdivider subdivider(graph, mediumData, lightDir, sampler, graphBuilder.GetSearchRadius(), config.subdivider, true);
    subdivider.ComputeSubdivisionEffect(lightVec);

    lighting.ComputeFinalLight(lightVec);

    // graph is in incorrect state after this
    graph.GetEdges().clear();

    std::string graphFileName = std::regex_replace(configName.value(), std::regex("\\.json"), ".txt");
    graph.WriteToDisk(graphFileName, graph::basic,
                      graph::StreamFlags{false, false, false, true});

    // graph::FreeGraph outline;
    // Bounds3f bounds = mediumData.primitiveData.bounds;
    // std::vector boundsMinMax = {bounds.pMin, bounds.pMax};
    // for (int x = 0; x < 2; ++x) {
    //     float xValue = boundsMinMax[x].x;
    //     for (int y = 0; y < 2; ++y) {
    //         float yValue = boundsMinMax[y].y;
    //         for (int z = 0; z < 2; ++z) {
    //             float zValue = boundsMinMax[z].z;
    //             outline.AddVertex({xValue, yValue, zValue}, graph::VertexData{});
    //         }
    //     }
    // }
    // auto SameSign = [](float a, float b) {
    //     return (a < 0 && b < 0) || (a >= 0 && b >= 0);
    // };
    // for (auto& [id1, v1] : outline.GetVertices()) {
    //     for (auto& [id2, v2] : outline.GetVertices()) {
    //         int numSame = 0;
    //         for (int i = 0; i < 3; ++i)
    //             if (SameSign(v1.point[i], v2.point[i]))
    //                 ++numSame;
    //         if (numSame == 2)
    //             outline.AddEdge(id1, id2, graph::EdgeData{});
    //     }
    // }
    // outline.WriteToDisk("C:/Users/tsvdh/CodeProjects/VSCode/PbrtScenes/cube/outline.txt", graph::outline, graph::StreamFlags{});

    CleanupPBRT();
    exit(0);
}
