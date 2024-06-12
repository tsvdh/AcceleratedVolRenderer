#pragma once

#include <pbrt/graph/graph.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "pbrt/util/args.h"
#include "pbrt/scene.h"

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

    Medium medium = scene.CreateMedia()["cloud"];
    Camera camera = scene.GetCamera();

    SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(0.5);

    Point2i pPixel(600, 350);

    CameraSample cameraSample;
    cameraSample.pFilm = pPixel + Vector2f(0.5f, 0.5f);
    cameraSample.time = 0.5f;
    cameraSample.pLens = Point2f(0.5f, 0.5f);
    cameraSample.filterWeight = 1;

    Ray ray = camera.GenerateRay(cameraSample, lambda).value().ray;

    ScratchBuffer buffer;
    RayMajorantIterator iter = medium.SampleRay(ray, Infinity, lambda, buffer);
    Point3f pointOnMedium = ray(iter.Next().value().tMin);

    NamedTextures textures = scene.CreateTextures();
    std::map<int, pstd::vector<Light>*> shapeIndexToAreaLights;
    std::vector<Light> lights = scene.CreateLights(textures, &shapeIndexToAreaLights);

    if (lights.size() != 1){
        std::cout << "Error: expected one light source" << std::endl;
        return 0;
    }

    Light light = lights[0];
    if (light.Type() != LightType::DeltaDirection) {
        std::cout << "Error: expected a directional light" << std::endl;
    }
    auto distantLight = light.Cast<DistantLight>();

    Vector3f lightDir = Normalize(distantLight->GetRenderFromLight()(Vector3f(0, 0, 1)));
    std::cout << lightDir << std::endl;

    // graph::FreeGraph graph;
    // graph.AddVertex();

    return 0;
}
