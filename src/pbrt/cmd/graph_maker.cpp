#include <pbrt/scene.h>
#include <pbrt/graph/graph.h>
#include <pbrt/graph/vol_transmittance.h>
#include <pbrt/util/args.h>

#include <iostream>
#include <regex>

#include "pbrt/graph/lighting_calculator.h"
#include "pbrt/graph/vol_boundary.h"

using namespace pbrt;

void main(int argc, char* argv[]) {
    std::vector<std::string> args = GetCommandLineArguments(argv);

    if (args.size() != 3) {
        ErrorExit("Expected exactly three arguments");
    }

    std::string mode = args[1];
    int wantedVertices = -1;
    float spacing = -1;

    if (mode == "wantedVertices")
        wantedVertices = std::stoi(args[2]);
    else if (mode == "spacing")
        spacing = std::stof(args[2]);
    else
        ErrorExit("Illegal mode argument");

    args.pop_back();
    args.pop_back();

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

    SampledWavelengths lambda = scene.GetCamera().GetFilm().SampleWavelengths(0.05);

    util::MediumData mediumData(lambda, accel);
    light->Preprocess(mediumData.bounds);

    Sampler sampler = scene.GetSampler();

    graph::VolBoundary boundary(mediumData);

    graph::UniformGraph boundaryGraph;
    if (wantedVertices != -1)
        boundaryGraph = boundary.CaptureBoundary(wantedVertices, 45, 45);
    else if (spacing != -1)
        boundaryGraph = boundary.CaptureBoundary(spacing, 45, 45);

    graph::UniformGraph transmittanceGrid = boundary.FillInside(boundaryGraph);

    graph::VolTransmittance transmittance(boundaryGraph, mediumData, light, sampler);
    transmittance.CaptureTransmittance(transmittanceGrid, 1);

    graph::LightingCalculator lighting(transmittanceGrid, mediumData, light, sampler);
    graph::UniformGraph finalLighting = lighting.GetFinalLightGrid(1, 4, 50);

    std::string fileName = std::regex_replace(args[0], std::regex("\\.pbrt"), ".txt");
    finalLighting.WriteToDisk(fileName, graph::grid_lighting,
        graph::StreamFlags{false, false, false, true});
}