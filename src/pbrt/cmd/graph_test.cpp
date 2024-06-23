#pragma once

#include <pbrt/graph/graph.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <pbrt/util/args.h>
#include <pbrt/scene.h>
#include <pbrt/cpu/aggregates.h>

#include <pbrt/graph/vol_boundary.h>

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

    // if (lights.size() != 1)
    //     throw std::runtime_error("Expected exactly one light source");
    //
    // Light light = lights[0];
    // if (!light.Is<DistantLight>())
    //     throw std::runtime_error("Expected a directional light");
    //
    // auto distantLight = light.Cast<DistantLight>();
    // Vector3f lightDir = -Normalize(distantLight->GetRenderFromLight()(Vector3f(0, 0, 1)));
    // Vector3f xVector;
    // Vector3f yVector;
    // CoordinateSystem(lightDir, &xVector, &yVector);

    std::map<std::string, pbrt::Material> namedMaterials;
    std::vector<pbrt::Material> materials;
    scene.CreateMaterials(textures, &namedMaterials, &materials);
    Primitive accel = scene.CreateAggregate(textures, shapeIndexToAreaLights, media, namedMaterials, materials);

    Camera camera = scene.GetCamera();
    SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(0.5);

    graph::VolBoundary boundary(accel, lambda);

    // --- cube ---
    // auto cubeGraph = boundary.CaptureBoundary(0.5, 40, 40);
    // cubeGraph->WriteToDisk("surface_cube", "surface");
    // auto cubeGraph = graph::UniformGraph::ReadFromDisk("surface_cube");
    // boundary.ToSingleLayer(cubeGraph);
    // ---

    // --- disney ---
    // auto disneyGraph = boundary.CaptureBoundary(10, 40, 40);
    // disneyGraph->WriteToDisk("surface_disney", "surface");
    auto disneyGraph = graph::UniformGraph::ReadFromDisk("surface_disney");
    boundary.ToSingleLayer(disneyGraph);
    disneyGraph->WriteToDisk("surface_disney_clean", "surface");
    // ---

    return 0;
}
