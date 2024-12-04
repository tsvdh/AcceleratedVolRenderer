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
#include "pbrt/graph/voxels/voxel_boundary.h"

using namespace pbrt;

void main(int argc, char* argv[]) {
    std::vector<std::string> args = GetCommandLineArguments(argv);

    std::string configName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".json");
    std::ifstream configStream(configName);
    nlohmann::json config = nlohmann::json::parse(configStream);

    auto graphBuilderConfig = config["graphBuilder"];
    auto lightingCalculatorConfig = config["lightingCalculator"];

    int dimensionSteps = graphBuilderConfig["dimensionSteps"];
    int maxDepth = graphBuilderConfig["maxDepth"];
    int edgeIterations = graphBuilderConfig["edgeIterations"];
    int lightIterations = lightingCalculatorConfig["lightIterations"];
    int transmittanceIterations = lightingCalculatorConfig["transmittanceIterations"];

    int maxSampleDimensionSize = std::sqrt(Sqr(dimensionSteps) * maxDepth);
    int pixelSamples = RoundUpPow2(std::max(edgeIterations, lightIterations));

    PBRTOptions options;
    options.disablePixelJitter = true;
    options.disableWavelengthJitter = true;
    options.renderingSpace = RenderingCoordinateSystem::World;
    options.pixelSamples = pixelSamples;
    options.graphSamplingResolution = Point2i(maxSampleDimensionSize, maxSampleDimensionSize);
    InitPBRT(options);

    BasicScene scene;
    BasicSceneBuilder builder(&scene);
    ParseFiles(&builder, args);

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
    light->Preprocess(mediumData.bounds);

    Sampler sampler = scene.GetSampler();

    graph::FreeGraphBuilder graphBuilder(mediumData, light, sampler);
    graph::FreeGraph graph = graphBuilder.TracePaths(dimensionSteps, maxDepth);
    graphBuilder.ComputeTransmittance(graph, edgeIterations);

    graph::FreeLightingCalculator lighting(graph, mediumData, light, sampler, lightIterations);
    lighting.ComputeFinalLight(transmittanceIterations);

    // graph is in incorrect state after this
    graph.GetEdges().clear();

    std::string fileName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".txt");
    graph.WriteToDisk(fileName, graph::basic,
        graph::StreamFlags{false, false, false, true});
}
