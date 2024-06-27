#pragma once

#include <pbrt/graph/graph.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <pbrt/util/args.h>
#include <pbrt/scene.h>
#include <pbrt/cpu/aggregates.h>

#include <pbrt/graph/vol_boundary.h>
#include <pbrt/graph/vol_transmittance.h>

using namespace pbrt;

int main(int argc, char* argv[]) {
    std::vector<std::string> args = GetCommandLineArguments(argv);

    if (args.size() != 1) {
        std::cout << "Error: expected exactly one argument" << std::endl;
        return 0;
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

    std::map<std::string, pbrt::Material> namedMaterials;
    std::vector<pbrt::Material> materials;
    scene.CreateMaterials(textures, &namedMaterials, &materials);
    Primitive accel = scene.CreateAggregate(textures, shapeIndexToAreaLights, media, namedMaterials, materials);

    Camera camera = scene.GetCamera();
    SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(0.5);

    auto mediumData = new util::MediumData(lambda, accel);

    graph::VolBoundary boundary(mediumData);

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

    graph::VolTransmittance transmittance(cubeGraph, mediumData);
    transmittance.CaptureTransmittance(lights);

    return 0;
}
