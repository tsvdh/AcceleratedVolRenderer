#pragma once

#include <pbrt/graph/graph.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <pbrt/util/args.h>
#include <pbrt/scene.h>
#include <pbrt/cpu/aggregates.h>

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

    // TODO: fix scene assumptions
    auto medium = media["cloud"].Cast<NanoVDBMedium>();
    Bounds3f mediumBounds = medium->GetRenderFromMedium()(medium->GetBounds());
    Point3f boundsMiddle = mediumBounds.pMin + mediumBounds.Diagonal() / 2;
    float maxDistToMiddle = Length(mediumBounds.Diagonal() / 2);

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

    Vector3f lightDir = -Normalize(distantLight->GetRenderFromLight()(Vector3f(0, 0, 1)));
    Vector3f xVector;
    Vector3f yVector;
    CoordinateSystem(lightDir, &xVector, &yVector);

    Camera camera = scene.GetCamera();
    SampledWavelengths lambda = camera.GetFilm().SampleWavelengths(0.5);

    graph::FreeGraph graph;

    Point3f origin(boundsMiddle - lightDir * maxDistToMiddle);
    graph.AddVertex(origin);
    // graph.AddVertex(origin + lightDir * 10);
    // graph.AddVertex(origin + xVector);
    // graph.AddVertex(origin + yVector);

    int numSteps = 100;
    float stepSize = maxDistToMiddle / (float)numSteps;
    xVector *= stepSize;
    yVector *= stepSize;

    int numRays = (int)std::pow(2 * numSteps, 2);
    int numHit = 0;

    for (int i = -numSteps; i < numSteps; ++i) {
        for (int j = -numSteps; j < numSteps; ++j) {
            Point3f newOrigin = origin + xVector * i + yVector * j;
            Ray gridRay(newOrigin, lightDir);

            auto iter = medium->SampleRay(gridRay, Infinity, lambda);
            while (true) {
                pstd::optional<RayMajorantSegment> segment = iter.Next();
                if (!segment)
                    break;
                if (segment->sigma_maj[0] != 0) {
                    graph.AddVertex(gridRay(segment->tMin));
                    numHit++;
                    break;
                }
            }
        }
    }

    std::cout << numHit << " / " << numRays << " | " << ((float)numHit / (float)numRays) << std::endl;

    graph.WriteToDisk("light_surface", "surface");

    return 0;
}
