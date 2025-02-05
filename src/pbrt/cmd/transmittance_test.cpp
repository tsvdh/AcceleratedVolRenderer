#include <climits>

#include "pbrt/lights.h"
#include "pbrt/pbrt.h"
#include "pbrt/scene.h"
#include "pbrt/graph/util.h"
#include "pbrt/graph/deps/json.hpp"

using namespace pbrt;

void main(int argc, int* argv[]) {
    static_assert(sizeof(double) * CHAR_BIT == 64, "Double must be 64 bits");

    int iterations = 8192;
    std::vector maxDepths = {1, 2, 3, 5, 10, 50, 100};

    int maxSampleDimensionSize = std::ceil(std::sqrt(maxDepths.size()));

    PBRTOptions options;
    options.disablePixelJitter = true;
    options.disableWavelengthJitter = true;
    options.renderingSpace = RenderingCoordinateSystem::World;
    options.pixelSamples = iterations;
    options.graph.samplingResolution = Point2i(maxSampleDimensionSize, maxSampleDimensionSize);

    InitPBRT(options);

    BasicScene scene;
    BasicSceneBuilder builder(&scene);
    ParseFiles(&builder, {"C:/Users/tsvdh/CodeProjects/VSCode/PbrtScenes/cube/cube.pbrt"});

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

    for (int a = 0; a < maxDepths.size(); ++a) {
        int totalScattered = 0;

        int yCoor = a / Options->graph.samplingResolution->x;
        int xCoor = a - yCoor * Options->graph.samplingResolution->x;

        for (int i = 0; i < iterations; ++i) {
            sampler.StartPixelSample({xCoor, yCoor}, i);

            Ray ray({0, 0, 0}, {1, 0, 0}, 0, mediumData.medium);
            int timesScattered = 0;

            RNG rng(Hash(sampler.Get1D()), Hash(sampler.Get1D()));
            SampleT_maj(ray, 1, sampler.Get1D(), rng, mediumData.defaultLambda,
                [&](Point3f p, MediumProperties mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {
                // Compute medium event probabilities for interaction
                Float pAbsorb = mp.sigma_a[0] / sigma_maj[0];
                Float pScatter = mp.sigma_s[0] / sigma_maj[0];
                Float pNull = std::max<Float>(0, 1 - pAbsorb - pScatter);

                CHECK_GE(1 - pAbsorb - pScatter, -1e-6);
                // Sample medium scattering event type and update path
                Float um = rng.Uniform<Float>();
                int mode = SampleDiscrete({pAbsorb, pScatter, pNull}, um);

                if (mode == 0) {
                    // Handle absorption along ray path
                    ErrorExit("No absorption possible");
                }
                if (mode == 1) {
                    // Handle scattering along ray path
                    return ++timesScattered < maxDepths[a];
                }
                else {
                    // Handle null scattering along ray path
                    ErrorExit("No null scattering possible");
                }
            });

            totalScattered += timesScattered;
        }

        float averageScattered = static_cast<float>(totalScattered) / static_cast<float>(iterations);
        std::cout << maxDepths[a] << " " << averageScattered << std::endl;
    }
}