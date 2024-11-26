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
#include "pbrt/graph/voxels/voxel_lighting_calculator.h"

using namespace pbrt;

void main(int argc, char* argv[]) {
    std::vector<std::string> args = GetCommandLineArguments(argv);

    PBRTOptions options;
    options.disablePixelJitter = true;
    options.disableWavelengthJitter = true;
    options.renderingSpace = RenderingCoordinateSystem::World;
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

    using json = nlohmann::json;

    std::string configName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".json");
    std::ifstream configStream(configName);
    json config = json::parse(configStream);

    // graph::VoxelBoundary boundary(mediumData);
    //
    // graph::UniformGraph boundaryGraph;
    // // boundaryGraph = boundary.CaptureBoundary(100, 45);
    // boundaryGraph = boundary.CaptureBoundary(0.5f, 45);
    //
    // graph::UniformGraph transmittanceGrid = boundary.FillInside(boundaryGraph);
    //
    // graph::VoxelTransmittance transmittance(boundaryGraph, mediumData, sampler);
    // transmittance.CaptureTransmittance(transmittanceGrid, 2, 2);
    //
    // graph::VoxelLightingCalculator lighting(transmittanceGrid, mediumData, light, sampler, 1000);
    // lighting.GetFinalLightGrid(100);
    //
    // std::string fileName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".txt");
    // transmittanceGrid.WriteToDisk(fileName, graph::grid_lighting,
    //     graph::StreamFlags{false, false, false, true});

    graph::FreeGraphBuilder graphBuilder(mediumData, light, sampler);
    graph::FreeGraph graph = graphBuilder.TracePaths(config["graphBuilder"]["dimensionSteps"], config["graphBuilder"]["maxDepth"]);
    graphBuilder.ComputeTransmittance(graph, config["graphBuilder"]["edgeIterations"]);

    graph::FreeLightingCalculator lighting(graph, mediumData, light, sampler, config["lightingCalculator"]["lightIterations"]);
    lighting.ComputeFinalLight(config["lightingCalculator"]["transmittanceIterations"]);

    // graph is in incorrect state after this
    graph.GetEdges().clear();

    std::string fileName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".txt");
    graph.WriteToDisk(fileName, graph::basic,
        graph::StreamFlags{false, false, false, true});
}
