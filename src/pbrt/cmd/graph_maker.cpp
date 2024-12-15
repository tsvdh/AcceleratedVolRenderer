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

void main(int argc, char* argv[]) {
    std::vector<std::string> args = GetCommandLineArguments(argv);

    std::string configName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".json");
    std::ifstream configStream(configName);
    nlohmann::json jsonConfig = nlohmann::json::parse(configStream);

    auto config = jsonConfig.get<graph::Config>();

    int maxDimensionSteps = std::max(config.graphBuilder.dimensionSteps,
                                     config.subdivider.graphBuilder.dimensionSteps);
    int maxMaxDepth = std::max(config.graphBuilder.maxDepth,
                               config.subdivider.graphBuilder.maxDepth);
    int maxIterations = std::max(
        std::max(config.graphBuilder.edgeIterations, config.lightingCalculator.lightIterations),
        std::max(config.subdivider.graphBuilder.edgeIterations, config.subdivider.lightingCalculator.lightIterations)
    );

    int numRays = Sqr(maxDimensionSteps);
    int maxSampleDimensionSize = std::ceil(std::sqrt(numRays * maxMaxDepth));
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

    std::string fileName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".txt");
    graph.WriteToDisk(fileName, graph::basic,
                      graph::StreamFlags{false, false, false, true});
}
