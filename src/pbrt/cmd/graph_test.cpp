#pragma once

#include <iomanip>
#include <iostream>

#include <pbrt/graph/graph.h>
#include <pbrt/scene.h>
#include <pbrt/graph/vol_transmittance.h>
#include <pbrt/util/args.h>

#include "pbrt/graph/vol_boundary.h"

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

    Sampler sampler = scene.GetSampler();

    graph::VolBoundary boundary(mediumData);

    // --- cube ---
    // auto cubeGraph = boundary.CaptureBoundary(0.5, 40, 40);
    // boundary.ToSingleLayer(cubeGraph);
    // cubeGraph->WriteToDisk("surfaces/cube", graph::Description::surface,
    //     graph::StreamFlags{false, false});

    auto cubeGraph = graph::UniformGraph::ReadFromDisk("surfaces/cube");
    // ---

    // --- disney ---
    // auto disneyGraph = boundary.CaptureBoundary(10, 40, 40);
    // boundary.ToSingleLayer(disneyGraph);
    // disneyGraph->WriteToDisk("surface_disney", graph::Description::basic,
    //     graph::StreamFlags{false, false});

    // auto disneyGraph = graph::UniformGraph::ReadFromDisk("surface_disney");
    // ---

    graph::VolTransmittance transmittance(cubeGraph, mediumData, sampler);
    graph::FreeGraph* paths = transmittance.CaptureTransmittance(lights);

    paths->WriteToDisk("paths/cube", graph::Description::paths,
        graph::StreamFlags{false, true, true});
}
