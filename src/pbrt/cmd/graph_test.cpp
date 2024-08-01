#pragma once

#include <iomanip>
#include <iostream>

#include <pbrt/graph/graph.h>
#include <pbrt/scene.h>
#include <pbrt/graph/vol_transmittance.h>
#include <pbrt/util/args.h>

using namespace pbrt;

void main(int argc, char* argv[]) {
    std::vector<std::string> args = GetCommandLineArguments(argv);

    if (args.size() != 1) {
        std::cout << "Error: expected exactly one argument" << std::endl;
        return;
    }

    PBRTOptions options;
    options.disablePixelJitter = true;
    options.disableWavelengthJitter = true;
    InitPBRT(options);

    BasicScene scene;
    BasicSceneBuilder builder(&scene);
    ParseFiles(&builder, args);

    std::map<std::string, Medium> media = scene.CreateMedia();

    NamedTextures textures = scene.CreateTextures();
    std::map<int, pstd::vector<Light>*> shapeIndexToAreaLights;
    std::vector<Light> lights = scene.CreateLights(textures, &shapeIndexToAreaLights);

    std::map<std::string, Material> namedMaterials;
    std::vector<Material> materials;
    scene.CreateMaterials(textures, &namedMaterials, &materials);
    Primitive accel = scene.CreateAggregate(textures, shapeIndexToAreaLights, media, namedMaterials, materials);

    Camera camera = scene.GetCamera();
    SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(0.5);

    auto mediumData = new util::MediumData(lambda, accel);

    // graph::VolBoundary boundary(mediumData);

    // --- cube ---
    // auto cubeGraph = boundary.CaptureBoundary(0.5, 40, 40);
    // cubeGraph->WriteToDisk("surface_cube", graph::Description::surface);
    auto cubeGraph = graph::UniformGraph::ReadFromDisk("surface_cube");
    // boundary.ToSingleLayer(cubeGraph);
    // cubeGraph->WriteToDisk("surface_cube", graph::Description::surface);
    // ---

    // --- disney ---
    // auto disneyGraph = boundary.CaptureBoundary(10, 40, 40);
    // disneyGraph->WriteToDisk("surface_disney", "surface");
    // auto disneyGraph = graph::UniformGraph::ReadFromDisk("surface_disney");
    // boundary.ToSingleLayer(disneyGraph);
    // disneyGraph->WriteToDisk("surface_disney", graph::Description::basic);
    // ---

    Sampler sampler = scene.GetSampler();

    graph::VolTransmittance transmittance(cubeGraph, mediumData, sampler);
    graph::FreeGraph* paths = transmittance.CaptureTransmittance(lights);

    paths->WriteToDisk("cube_paths", graph::Description::paths);
}
